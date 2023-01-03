import requests
import os
job_token = os.environ["CI_JOB_TOKEN"]
binary_name = os.environ["CI_JOB_TOKEN"]
UBUNTU_BINARY = os.environ["UBUNTU_BINARY"]
whisper_url = f"{os.environ['PACKAGE_REGISTRY_URL']}/{UBUNTU_BINARY}"
ralph_url = f"{os.environ['RALPH_PACKAGE_REGISTRY_URL']}/{UBUNTU_BINARY}"


def upload_file(url: str):
    file1 = dict(file=open(f"build-Linux/{UBUNTU_BINARY}", 'rb'))

    requests.post(url=url, header={"JOB-TOKEN": os.environ['CI_JOB_TOKEN']}, files=file1)


upload_file(whisper_url)
upload_file(ralph_url)
