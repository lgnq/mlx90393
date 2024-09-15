from building import *
Import('rtconfig')

src   = []
cwd   = GetCurrentDir()

# add mlx90393 src files.
if GetDepend('PKG_USING_MLX90393'):
    src += Glob('src/mlx90393.c')

if GetDepend('PKG_MLX90393_USING_SENSOR_V1'):
    src += Glob('src/melexis_mlx90393_sensor_v1.c')

if GetDepend('PKG_USING_MLX90393_SAMPLE'):
    src += Glob('examples/mlx90393_sample.c')

# add mlx90393 include path.
path  = [cwd + '/inc']

# add src and include to group.
group = DefineGroup('mlx90393', src, depend = ['PKG_USING_MLX90393'], CPPPATH = path)

Return('group')
