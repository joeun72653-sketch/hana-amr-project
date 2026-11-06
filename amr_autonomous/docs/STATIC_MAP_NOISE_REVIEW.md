# Global Costmap 내부 점노이즈 후보 제거 (원본 보존)

목표: 벽과 실제 구조물은 유지하고, 저장된 지도 내부의 **아주 작고 독립적인 점형 장애물**만 검토 대상으로 추려 Global Costmap에서 불필요한 Inflation이 발생하는지 확인합니다. 이것은 Costmap 자체를 끄거나 Footprint를 줄이는 작업이 아닙니다.

## 이전 검증에서 확인된 점

- Global Costmap: static_layer + obstacle_layer + inflation_layer
- Global inflation_radius: 0.50m로 임시 조정; Local inflation_radius 0.55m 그대로
- Footprint: 0.87m × 0.72m; 로봇이 실제로 차지하는 영역은 지울 수 없음
- `obstacle_layer.scan.topic=/scan_slam`, marking=true, clearing=true
- `/global_costmap/static_layer`, `/global_costmap/obstacle_layer` 모두 제공
- RViz 스크린샷에 저장된 맵 내부 작은 점들이 보여 일부는 Static Layer 노이즈일 수 있지만, 실제 장애물 여부는 사진만으로 확정 불가

## 안전한 처리 순서

1. 기존 지도 YAML 이미지 형식을 확인합니다:
   ```bash
   cat ~/.ros/real_amr/mapex_maps/real_mapex_20261028_162015.yaml
   ```
2. PNG가 아닌 PGM 지도라면 아래 명령어로 **원본을 수정하지 않는 후보**를 만듭니다:
   ```bash
   export PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=31 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   source /opt/ros/jazzy/setup.bash
   source ~/real_amr/install/setup.bash
   python3 -u ~/hana-amr-project-autonomous/amr_autonomous/scripts/review_static_map_speckles.py \
      --map ~/.ros/real_amr/mapex_maps/real_mapex_20261028_162015.yaml \
      --max-cluster-pixels 4 --border-margin-m 0.4
   ```
3. 결과는 원본 YAML 옆의 `real_mapex_20261028_162015_review/` 디렉터리에 **별도로** 생성됩니다.
   - `map_speckle_candidate.pgm`: 작은 점 후보만 free로 변경한 복사본
   - `map_speckle_candidate.yaml`: 복사본 지도 참조
   - `removed_pixels_review.pgm`: 검은 픽셀은 제거 후보
   - `review.json`: 제거 개수와 각 후보 중심 좌표
4. `removed_pixels_review.pgm`을 이미지 뷰어에서 열고 모든 제거 후보를 확인합니다. 실제 안전시설, 물체, 사람, 가구, 위험 물체가 포함되면 해당 후보는 제거하지 말고 원본을 유지해야 합니다.
5. 현장 확인이 끝나기 전에는 Nav2 `map_file`을 후보 YAML로 바꾸지 않습니다. Local Costmap의 실시간 LiDAR 장애물 인식은 계속 유지해야 합니다.

**중요:** 이 도구는 저장된 Static Layer의 아주 작은 고립 점만 대상으로 합니다. **실시간 Obstacle Layer의 LiDAR 허위 감지, 넓은 구조물, 벽, Costmap Inflation 자체를 제거하지 않습니다.** 큰 청록색 영역이 그대로 남으면 Global Obstacle Layer, 센서 마스킹/클리어링, 차체 Footprint와 지도의 불일치를 별도로 확인해야 합니다.

본 프로젝트의 `run_all.sh`는 변함없이 **원래 지도 YAML**을 사용합니다. 후보는 자동으로 주행에 사용되지 않습니다.
