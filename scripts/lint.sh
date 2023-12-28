echo "Running autopep8"
autopep8 --exclude extern -aaair .
echo "Running mypy"
mypy .
echo "Running flake8"
flake8 .
echo "Running pyright"
pyright .
echo "Converting all line endings"
find . -type f -exec dos2unix {} \; > /dev/null 2>&1
