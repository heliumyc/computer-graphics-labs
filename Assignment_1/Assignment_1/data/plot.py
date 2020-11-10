import pywavefront
from pywavefront import visualization

obj = pywavefront.Wavefront('polygon.obj')

visualization.draw(obj)
input()
