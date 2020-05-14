import os
import sys
sys.path.insert(0, os.path.abspath('../scripts'))

project = 'spot_driver'
copyright = '2020, Dave Niewinski'
author = 'Dave Niewinski'

extensions = [
'sphinx.ext.autodoc',
'sphinx.ext.coverage',
'sphinx.ext.napoleon'
]

html_theme = 'clearpath-sphinx-theme'
html_theme_path = ["."]

html_static_path = ['./clearpath-sphinx-theme/static']

html_sidebars = {
   '**': ['sidebartoc.html', 'sourcelink.html', 'searchbox.html']
}

html_show_sphinx = False

html_logo = 'clearpath-sphinx-theme/static/clearpathlogo.png'

html_favicon = 'clearpath-sphinx-theme/static/favicon.ico'
