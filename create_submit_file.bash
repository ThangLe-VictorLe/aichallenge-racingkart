#!/bin/bash
set -e

echo "Preparing submission..."

# Create a temporary directory
TMP_DIR=$(mktemp -d)
cp -r ./aichallenge/workspace/src/aichallenge_submit $TMP_DIR/aichallenge_submit

# Run the obfuscation script on the temp directory
python3 ./obfuscate_configs.py $TMP_DIR/aichallenge_submit

# Tar the obfuscated directory
mkdir -p submit
tar zcvf submit/aichallenge_submit.tar.gz -C $TMP_DIR aichallenge_submit

# Clean up
rm -rf $TMP_DIR
echo "Created submit/aichallenge_submit.tar.gz successfully with hidden configs."
