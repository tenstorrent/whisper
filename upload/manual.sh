package_name="swerv-iss"
whisper_id="22"
ralph_id="104"
os_name="ubuntu"
release_name=$(git tag --points-at HEAD)



curl --header "PRIVATE-TOKEN: Jzq77CkSHSSvQLrw2FxE"  --upload-file "build-Linux/whisper"  "https://aus-gitlab.local.tenstorrent.com/api/v4/projects/${ralph_id}/packages/generic/${package_name}/${release_name}/whisper"
curl --header "PRIVATE-TOKEN: Jzq77CkSHSSvQLrw2FxE"  --upload-file "build-Linux/whisper"  "https://aus-gitlab.local.tenstorrent.com/api/v4/projects/${whisper_id}/packages/generic/${package_name}/${release_name}/whisper"
