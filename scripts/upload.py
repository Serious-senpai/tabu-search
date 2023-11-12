import os
import sys
from urllib.request import Request, urlopen


token = os.environ["HARUKA_TOKEN"]
path = sys.argv[1]


with open(path, "rb") as file:
    request = Request(
        url="https://haruka39.me/api/upload",
        data=file.read(),
        headers={"X-Auth-Token": token},
        method="POST",
    )


urlopen(request)
