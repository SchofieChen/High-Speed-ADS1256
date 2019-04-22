from distutils.core import setup, Extension

c_ext = Extension("SMG_ADS1256PyLib", ["SMG_ADS1256PyLib.cpp"], libraries = ['bcm2835'])

setup(
    ext_modules=[c_ext],
)
