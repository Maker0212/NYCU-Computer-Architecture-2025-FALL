#!/bin/bash
# Step 1: rename files

rsync -av --exclude=setup.sh ../riscvdualfetch/ .

for file in *; do
  if [[ "$file" == *riscvdualfetch* ]]; then
    mv "$file" "${file/riscvdualfetch/riscvssc}"
  fi
done

# Step 2: replace inside files
for file in *; do
  [ "$file" = "setup.sh" ] && continue
  if [ "$(uname)" = "Darwin" ]; then
    sed -i '' 's/riscvdualfetch/riscvssc/g' "$file"
  else
    sed -i 's/riscvdualfetch/riscvssc/g' "$file"
  fi
done
