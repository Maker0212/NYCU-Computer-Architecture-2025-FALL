#!/bin/bash
# Step 1: rename files

rsync -av --exclude=setup.sh ../riscvstall/ .

for file in *; do
  if [[ "$file" == *riscvstall* ]]; then
    mv "$file" "${file/riscvstall/riscvbyp}"
  fi
done

# Step 2: replace inside files
for file in *; do
  [ "$file" = "setup.sh" ] && continue
  if [ "$(uname)" = "Darwin" ]; then
    sed -i '' 's/riscvstall/riscvbyp/g' "$file"
  else
    sed -i 's/riscvstall/riscvbyp/g' "$file"
  fi
done
