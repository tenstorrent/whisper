import requests
import os
job_token = os.env["CI_JOB_TOKEN"]
binary_name = os.env["CI_JOB_TOKEN"]
UBUNTU_BINARY = os.env["UBUNTU_BINARY"]
whisper_url = f"{os.env['PACKAGE_REGISTRY_URL']}/{UBUNTU_BINARY}"
ralph_url = f"{os.env['RALPH_PACKAGE_REGISTRY_URL']}/{UBUNTU_BINARY}"


def upload_file(url: str):
    file1 = dict(file=open(f"build-Linux/{UBUNTU_BINARY}", 'rb'))

    requests.post(url=url, header={"JOB-TOKEN": os.env['CI_JOB_TOKEN']}, files=file1)


upload_file(whisper_url)
upload_file(ralph_url)
