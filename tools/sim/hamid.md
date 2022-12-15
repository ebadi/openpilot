openpilot cleanup
=====================
```
git clone repo
git submodule update --init --recursive
cd openpilot/tools/sim
./build_container.sh
./start_carla.sh

./start_openpilot_docker.sh
```

if `./ui.py` is not working, 
```
pip uninstall opencv-python-headless
pip install opencv-python-headless 
```


to kill:
```
docker kill openpilot_client  ; pkill carla
```
