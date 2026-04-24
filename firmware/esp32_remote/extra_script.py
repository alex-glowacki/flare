# type: ignore      #SCons globals injected by PlatformIO at build time
import os
Import("env")       # noqa: F821

shared_path = os.path.abspath(
    os.path.join(env["PROJECT_DIR"], "..", "shared")
)
env.Append(CPPPATH=[shared_path])