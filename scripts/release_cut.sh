#!/bin/bash
# Cut a new release: bump the RPM spec version to YYYY.MM.P, commit it,
# tag the commit as vYYYY.MM.P and push both to the release branch.

set -euo pipefail

RELEASE_BRANCH="main"

script_dir="$(dirname "$(readlink -f "$0")")"
repo_dir="$(readlink -f "$script_dir/..")"
cd "$repo_dir"

echo "This will create a new commit on top of the current HEAD, tag it and"
echo "push it to origin/$RELEASE_BRANCH."
read -r -p "Continue? [y/N] " answer
if [[ "$answer" != "y" && "$answer" != "Y" ]]; then
  echo "Aborted."
  exit 1
fi

current_branch="$(git rev-parse --abbrev-ref HEAD)"
if [[ "$current_branch" != "$RELEASE_BRANCH" ]]; then
  echo "Error: must be on '$RELEASE_BRANCH' branch (currently on '$current_branch')." >&2
  exit 1
fi

git pull --ff-only
git fetch --tags

year="$(date +%Y)"
month="$(date +%m)"
prefix="v${year}.${month}."

latest_patch=0
while read -r tag; do
  patch="${tag#"$prefix"}"
  if [[ "$patch" =~ ^[0-9]+$ ]] && ((10#$patch > latest_patch)); then
    latest_patch=$((10#$patch))
  fi
done < <(git tag -l "${prefix}*")

version="${year}.${month}.$((latest_patch + 1))"
tag="v${version}"

spec_files=("$repo_dir"/rpmbuild/*.spec)
if [[ ! -f "${spec_files[0]}" ]]; then
  echo "Error: no spec file found in $repo_dir/rpmbuild." >&2
  exit 1
fi

for spec in "${spec_files[@]}"; do
  sed -i -E "s/^Version:.*/Version: ${version}/" "$spec"
done

echo "Releasing version $version"

git add "${spec_files[@]}"
git commit -m "Release ${tag}" -- "${spec_files[@]}"
git tag -a "$tag" -m "Release ${tag}"
git push --atomic origin "$RELEASE_BRANCH" "$tag"

echo "Released $tag"
