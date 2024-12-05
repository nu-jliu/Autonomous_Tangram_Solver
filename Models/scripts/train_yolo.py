import os
from argparse import ArgumentParser

from roboflow import Roboflow
from ultralytics import YOLO

FILE_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(FILE_DIR, "..", "dataset", "yolo_shapes")
MODEL_DIR = os.path.join(FILE_DIR, "..", "model")


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("--num-epoch", default=1000000, help="Number of epochs")
    parser.add_argument("--lamb", action="store_true", help="Runs on lamb or sheep")
    parser.add_argument("--model-name", default="shapes", help="Name of the model file")

    args = parser.parse_args()
    num_epochs = int(args.num_epoch)
    batch_size = 128 if args.lamb else 32
    model_name = args.model_name

    rf = Roboflow(api_key="8CrldWKFJ6M2rJPp0f9f")
    project = rf.workspace("cell-fcwyr").project("shape-detection-tet2v")
    version = project.version(1)
    dataset = version.download("yolov11", location=DATA_DIR, overwrite=True)
    # model = YOLO("yolo11n-cls.pt", task="classify", verbose=True)
    model = YOLO("yolo11n.pt", task="detect", verbose=True)

    # results = model.train(
    #     data=DATA_DIR,
    #     batch=batch_size,
    #     epochs=num_epochs,
    #     imgsz=640,
    #     patience=1000,
    # )

    results = model.train(
        data=os.path.join(DATA_DIR, "data.yaml"),
        batch=batch_size,
        epochs=num_epochs,
        imgsz=640,
        patience=1000,
    )

    model.save(os.path.join(MODEL_DIR, f"{model_name}.pt"))
