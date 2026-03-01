from ultralytics import YOLO

model = YOLO('../models/trained/best.pt')
model.export(format='onnx', imgsz=640, simplify=True)
print("Model exported to ONNX format")
