# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

import os
import sys
<<<<<<< HEAD
#import pteranodon

sys.path.insert(0, os.path.abspath('..'))



project = 'Pteranodon'
copyright = '2022, Field Session'
author = 'Field Session'
=======
>>>>>>> 197ea54696175e840e0e4b10dd9d16ae938abc4a

# import pteranodon

sys.path.insert(0, os.path.abspath(".."))


project = "Pteranodon"
copyright = "2022, Field Session"
author = "Field Session"

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

<<<<<<< HEAD
extensions = ['sphinx.ext.autodoc',
              'sphinx.ext.napoleon',
              'sphinx.ext.viewcode']

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']
=======
>>>>>>> 197ea54696175e840e0e4b10dd9d16ae938abc4a
extensions = ["sphinx.ext.autodoc", "sphinx.ext.napoleon", "sphinx.ext.viewcode"]

templates_path = ["_templates"]
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]
<<<<<<< HEAD
=======

>>>>>>> 197ea54696175e840e0e4b10dd9d16ae938abc4a

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

<<<<<<< HEAD
html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
=======
>>>>>>> 197ea54696175e840e0e4b10dd9d16ae938abc4a
html_theme = "sphinx_rtd_theme"
html_static_path = ["_static"]
