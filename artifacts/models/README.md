# Models

`movement_gru.onnx` is the unchanged camera movement-profile model inherited
from LIMB1. `movement_gru.json` records its hash and input format. Its exact
original training split and evaluation results are unavailable. New training
runs save those details in `training.json`.

`cup_yolo11n.onnx` is Ultralytics YOLO11n, pretrained on COCO. It runs locally
through ONNX Runtime and keeps the `cup` class. Detection scores and boxes are
saved with camera observations. Missing or hidden objects can still be missed.

The weights come from the [official v8.3.0 release](https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt),
exported with Ultralytics 8.3.0 as ONNX, 640 x 640, opset 12, without simplification.
The model is supplied under [AGPL-3.0](ULTRALYTICS_LICENSE.txt).

SHA-256: `57005b53544a85d76a86f576c529f3cf7c8b4498c1a9cd04f89730974cb15886`.
