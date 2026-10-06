# Agent rules for this repo

## Build and test
- Install: `./build_linux.sh -u -d`
- Test (must pass before any push): `./build_linux.sh -st && (cd build && ctest -C Release --output-on-failure)`
- Lint: `none`
- Package (what CI publishes): `./build_linux.sh -i && mkdir -p dist && find build -maxdepth 4 -name '*.AppImage' -exec cp -v {} dist/ +` -> `dist/`

## Git
- Never commit to or push `main`. Work on `hermes/<short-topic>` branches.
- One logical change per branch. Rebase on `origin/main` before pushing.
- Before opening a PR, delegate an independent review of the full diff to a
  subagent, giving it the task, this file and the test output. Fix what it finds,
  re-run the tests, and summarise the review in the PR body.
- Open PRs with `gh pr create`. Body: what changed, why, the test output and the review summary.
- Use `--draft` unless all tests pass locally.
- `.github/workflows/` needs care: pushing those files requires the token's **Workflows** permission — the stage-05 default lacks it and git rejects the push. Where an operator has granted it (the kit's `docs/GITHUB-SETUP.md` §8), workflow edits follow the same PR review as everything else.

## Releases
- Version lives in: `version.inc`
- Changelog: `CHANGELOG.md`, newest section first.
- Semantic versioning. Tags are `vX.Y.Z` on the release PR's merge commit.
- Never delete or move a tag. Fix mistakes with a new patch release.

## Machine and boundaries
- You have passwordless sudo on this laptop. Install missing system packages with
  `sudo apt install`, and list each one in the PR body so the setup stays reproducible.
- Do not add project dependencies without saying why in the PR.
- Secrets never go in code, logs or PR text.
