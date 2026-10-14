# Evidence from running the project READMEs

These compact snapshots preserve the accepted run of each of the 17 project guides. See the [execution report](../../README_RUN_REPORT.md) for outcomes, repairs, failed attempts and limits. [results.tsv](results.tsv) maps each guide to its original artifact directory and accepted log. All exit codes are zero; expected negative tests may still contain error messages.

Each project folder contains the commands extracted from its README, its exercise log and its result record. CSV outputs, interpretation/training records and rejection logs are included where relevant. Absolute paths inside logs identify the checkout and original run; use the course setup instructions for another machine. Large backend logs, images, datasets and checkpoints remain in the Git-ignored `Learning Path/artifacts` directories listed in the manifest. Failed attempts remain there too and are not included as accepted snapshots.

[build.log](build.log) records the final build of three existing dependencies and nine course packages. [checks.log](checks.log) records the three passing CTests and launch/XML syntax checks. The recorder's `dataset_checks.tsv` additionally records dimensions, image availability, finite values and measured future-action horizons for every recorded sample.
