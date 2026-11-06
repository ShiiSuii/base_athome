#!/bin/bash
# Arma la rama local main-preview: main + el contenido de development sin lo listado en main_excluir.txt.
# No toca main ni empuja nada. Publicar a main es decision humana: revisar y empujar a mano con el comando que imprime.
#   tools/agents/publicar_main.sh [origen]     (por defecto origin/development)
set -eu
SRC=${1:-origin/development}
HERE=$(cd "$(dirname "$0")" && pwd)
cd "$(git rev-parse --show-toplevel)"
git fetch -q origin
IDX=$(mktemp -u)
trap 'rm -f "$IDX"' EXIT
export GIT_INDEX_FILE=$IDX
git read-tree "$SRC"
grep -vE '^(#|$)' "$HERE/main_excluir.txt" | while read -r p; do
  git rm -r -q --cached --ignore-unmatch -- "$p"
done
TREE=$(git write-tree)
SHA=$(git rev-parse --short "$SRC")
NEW=$(git commit-tree "$TREE" -p origin/main -m "Publish robot sources from development ($SHA)")
git branch -f main-preview "$NEW"
unset GIT_INDEX_FILE
echo "main-preview = $(git rev-parse --short main-preview), desde $SRC ($SHA)"
git diff --stat origin/main main-preview | tail -1
echo "Revisar: git diff --stat origin/main main-preview"
echo "Publicar, con autorizacion: git push origin main-preview:main"
