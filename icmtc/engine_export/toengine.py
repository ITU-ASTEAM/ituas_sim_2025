from ultralytics import YOLO

model = YOLO("best1JUL.pt")

model.export(format="engine",half=True, imgsz=640, device=0)
