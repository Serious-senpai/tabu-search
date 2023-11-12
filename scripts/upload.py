import os
import re
import sys
from urllib.request import Request, urlopen


token = os.environ["HARUKA_TOKEN"]
path = sys.argv[1]
name = "".join(c for c in sys.argv[2] if re.match(r"[a-zA-Z0-9_\-()\.]", c) is not None)
with open(path, "rb") as file:
    request = Request(
        url=f"https://haruka39.me/api/upload?name={name}&extract=1",
        data=file.read(),
        headers={"X-Auth-Token": token},
        method="POST",
    )


urlopen(request)
