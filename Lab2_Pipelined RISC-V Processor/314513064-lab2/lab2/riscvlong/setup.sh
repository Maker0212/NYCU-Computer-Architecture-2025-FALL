#!/bin/bash
# Step 1: rename files

rsync -av --exclude=setup.sh ../riscvbyp/ .

for file in *; do
  if [[ "$file" == *riscvbyp* ]]; then
    mv "$file" "${file/riscvbyp/riscvlong}"
  fi
done

# Step 2: replace inside files
for file in *; do
  [ "$file" = "setup.sh" ] && continue
  if [ "$(uname)" = "Darwin" ]; then
    sed -i '' 's/riscvbyp/riscvlong/g' "$file"
  else
    sed -i 's/riscvbyp/riscvlong/g' "$file"
  fi
done
