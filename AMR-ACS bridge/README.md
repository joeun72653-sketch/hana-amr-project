# AMR-ACS bridge

Ubuntu 24.04 / ROS 2 Jazzy AMR IPC ↔ ACS connectivity.

## Active IPC bridge (source of truth)

- File on AMR IPC: `~/amr_remote_client/amr_ws_to_ros_acs.py`
- Integrated launch: `~/amr_remote_client/amr_all_with_acs.launch.py`
- ACS connection config: `~/.config/amr/acs_connection.json` (**never commit**)
- ROS_DOMAIN_ID: `31`

The running IPC file must be copied from the robot before claiming a GitHub copy matches the active deployment. This folder is an index, **not a deployed or verified source snapshot**.

## Sync active code from Ubuntu

From a checked-out clone of this repository on the AMR:

```bash
mkdir -p 'AMR-ACS bridge'
cp ~/amr_remote_client/amr_ws_to_ros_acs.py 'AMR-ACS bridge/amr_ws_to_ros_acs.py'
cp ~/amr_remote_client/amr_all_with_acs.launch.py 'AMR-ACS bridge/amr_all_with_acs.launch.py'
python3 -m py_compile 'AMR-ACS bridge/amr_ws_to_ros_acs.py'
git add 'AMR-ACS bridge/amr_ws_to_ros_acs.py' 'AMR-ACS bridge/amr_all_with_acs.launch.py'
git commit -m 'AMR-ACS bridge: sync active IPC bridge and integrated launch'
git push
```

Before pushing, review code and staged changes for tokens, tickets, host credentials, passwords and private addresses. Do not add `acs_connection.json`.
