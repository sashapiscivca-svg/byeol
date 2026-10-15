# Security and misuse reporting

**English** · [Українська](SECURITY.uk.md)

## How to report

Report privately through GitHub: open the **Security** tab of this repository and choose **"Report a vulnerability"**. Only the maintainers can see the report. Please do not open public issues for:

- **vulnerabilities**, for example input that crashes the estimator or makes it return `valid=True` with a wildly wrong result and a small covariance;
- **safety-relevant defects**, meaning any case where byeol's output could mislead a rescue operation, such as overconfident covariance or wrong sign conventions;
- **misuse**, meaning use of byeol against the principles in [ETHICS.md](ETHICS.md).

Include what you observed, how to reproduce it (inputs, configuration, version) and its possible impact. We aim to acknowledge reports within 7 days.

## Scope

byeol is a research-preview estimator that only reads sensor data. It must not be used in flight control or safety loops ([MODEL_CARD.md](MODEL_CARD.md)). Reports about systems that do so anyway are still welcome, because they help us improve the warnings.

## Supported versions

| Version | Supported |
|---|---|
| 0.1.x | yes |
