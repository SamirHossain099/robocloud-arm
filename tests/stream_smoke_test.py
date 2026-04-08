import time

from pi.perception.camera import Camera
from pi.perception.stream import start_stream_server


def main() -> None:
    camera = Camera(index=0)
    camera.start()

    # Give camera a moment to warm up.
    time.sleep(0.5)

    print("Starting test stream on http://0.0.0.0:8080/stream")
    start_stream_server(camera=camera, host="0.0.0.0", port=8080)


if __name__ == "__main__":
    main()
