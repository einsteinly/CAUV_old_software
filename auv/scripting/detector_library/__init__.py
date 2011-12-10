from glob import glob
__all__ = [f[17:-3] for f in glob('detector_library/*.py') if f[17:-3]!='__init__']
