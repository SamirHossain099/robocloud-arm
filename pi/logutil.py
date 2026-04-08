import os


def verbose() -> bool:
    return os.getenv("ROBOCLOUD_VERBOSE", "0") == "1"


def vprint(*args, **kwargs) -> None:
    if verbose():
        print(*args, **kwargs)
