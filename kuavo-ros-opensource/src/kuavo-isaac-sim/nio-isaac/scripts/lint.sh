set -e

ruff .
flake8 .
pylint $(git ls-files '*.py')
