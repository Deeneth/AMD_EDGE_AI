from ultralytics import YOLO
import torch

if __name__ == '__main__':
    device = 0 if torch.cuda.is_available() else 'cpu'
    print(f"Using device: {'GPU' if device == 0 else 'CPU'}")
    
    model = YOLO('../models/yolov8n.pt')
    
    results = model.train(
        data='../data/Yolo model5/data.yaml',
        epochs=100,
        imgsz=640,
        batch=16,
        device=device,
        patience=20,
        dropout=0.2,
        augment=True,
        name='yolov8_vehicle_detection'
    )
