scriptpath=$( cd -P -- "$(dirname -- "$(command -v -- "$1")")" && pwd -P )

if [[ "$1" == "-f" ||  "$1" == "force" ]]; then
  if [ -d $scriptpath/ralph_venv ]; then
    rm -rf ralph_venv
  fi
fi

if [ ! -d $scriptpath/ralph_venv ]; then
  pip3 install --user virtualenv
  virtualenv ralph_venv
  python3 -m venv ralph_venv
  source ralph_venv/bin/activate
  pip install git+https://aus-gitlab.local.tenstorrent.com/arch/ralph.git@master
else
  source ralph_venv/bin/activate
fi
