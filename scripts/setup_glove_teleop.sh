#!/usr/bin/env bash
set -euo pipefail

ENV_NAME="gloveTeleop"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ENV_FILE="${REPO_ROOT}/environment.yml"

find_conda() {
  if command -v conda >/dev/null 2>&1; then
    command -v conda
    return 0
  fi

  for candidate in "${HOME}/miniconda3/bin/conda" "${HOME}/anaconda3/bin/conda" "/opt/conda/bin/conda"; do
    if [ -x "${candidate}" ]; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  done

  return 1
}

CONDA_BIN="$(find_conda)" || {
  echo "Could not find conda. Install Miniconda/Anaconda or add conda to PATH." >&2
  exit 1
}

if "${CONDA_BIN}" env list | awk '{print $1}' | grep -qx "${ENV_NAME}"; then
  echo "Updating existing ${ENV_NAME} environment..."
  "${CONDA_BIN}" env update -n "${ENV_NAME}" -f "${ENV_FILE}" --prune
else
  echo "Creating ${ENV_NAME} environment..."
  "${CONDA_BIN}" env create -f "${ENV_FILE}"
fi

cat <<EOF

Environment ready: ${ENV_NAME}

Activate it with:
  conda activate ${ENV_NAME}

Verify it with:
  conda run --no-capture-output -n ${ENV_NAME} python scripts/check_environment.py
EOF
