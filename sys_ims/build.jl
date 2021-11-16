using PackageCompiler
using RExQuad

PackageCompiler.create_sysimage(
    ["RExQuad"];
    sysimage_path="RExQuadSysImage.so"
)
