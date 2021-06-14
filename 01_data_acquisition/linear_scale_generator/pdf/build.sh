TIMEFORMAT=%3lU
SCRIPTPATH="$( cd "$(dirname "$0")" ; pwd -P )"
pushd $SCRIPTPATH > /dev/null

echo "building..."
time pdflatex -interaction=nonstopmode scale.tex || exit 1

popd > /dev/null
