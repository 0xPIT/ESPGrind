import subprocess

Import("env")

def get_git_version():
    ret = subprocess.run(["git", "describe", "--tags", "--dirty", "--always"], stdout=subprocess.PIPE, text=True)
    build_version = ret.stdout.strip()
    build_flag = "-DGIT_VERSION=\\\"" + build_version + "\\\""
    return (build_flag)

env.Append(
    BUILD_FLAGS=[get_git_version()]
)
