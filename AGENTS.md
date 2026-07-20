# blacknode-skills Agent Instructions

This is an independent Blacknode extension-package repository.

Keep reusable task and mission skills here. Consume stable robot capabilities;
never import physical driver SDKs or hard-code device paths, topics, frames, or
vendor models. Keep skills disarmed until controller authorization succeeds.
Declare every required package/component in `blacknode-package.toml` and test
skills with mock or replay providers before supported hardware providers.

Run package tests with `python -m pytest packages/blacknode-skills/tests`.
