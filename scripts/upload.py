import os
import sys
from urllib.parse import quote
from urllib.request import Request, urlopen

token = os.environ["HARUKA_TOKEN"]
path = sys.argv[1]
name = quote(sys.argv[2])


with open(path, "rb") as file:
    request = Request(
        url=f"https://haruka39.me/api/upload?name={name}&extract=1",
        data=file.read(),
        headers={"X-Auth-Token": token},
        method="POST",
    )


urlopen(request)
