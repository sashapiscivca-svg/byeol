# Responsible use

**English** · [Українська](ETHICS.uk.md)

> Version 1.0 · 2026-10-15 · Applies to byeol and the other tools of the *uju* family: hae (detection), dal (tracking), byeol (camera motion physics), together with their code, data and documentation.

## 1. Why we built this

We built these tools to **find people who need help**. That includes people swept away by floods or tsunamis, people lost in forests or wilderness, and people trapped after disasters. The tools can also count and locate livestock and wildlife in dangerous situations. We target low-cost hardware (Raspberry Pi class), so that volunteer rescue groups can use them, not only organisations with large budgets. We measure success in lives saved.

## 2. Legal status: please read

byeol is released under the **MIT License**. The license **does not restrict** how you use the software, and this document does not change that. Nothing here is a license condition.

This document is our **public position**. It sets out what we built byeol for and what we ask you not to do with it. We publish it openly because open code cannot be fully protected from misuse, and we would rather be honest about that than promise protection we cannot give.

## 3. What we ask you not to use it for

Please do not use byeol, its derivatives, or systems that include it, directly or indirectly, for:

1. **Military purposes.** This covers reconnaissance, detection, tracking or guidance against people or vehicles for combat, weapons or weapon control, target designation, and "defensive" or ISR applications.
2. **Persecution.** This means detecting, following or pressuring people because of their views, faith, origin, language, gender, orientation, or political or civic activity, including taking part in assemblies and protests.
3. **Mass or covert surveillance.** This means continuous monitoring of populations, or tracking people without their knowledge and without a lawful basis.
4. **Repression.** This includes use by any state or other actor to suppress rights and freedoms, censor, detain arbitrarily, torture, forcibly displace, or carry out ethnic or religious persecution.
5. **Acting against people who need protection.** For example, detecting refugees and migrants for unlawful pushbacks, or hunting people in distress instead of helping them.
6. **Identifying individuals.** This includes face recognition, biometrics, and inferring group membership from appearance.
7. **Any use intended or foreseeably likely to cause death, injury, coercion or intimidation.**

If you are unsure whether a use helps or harms people, assume it harms them until you have shown otherwise.

## 4. Uses we consider appropriate

- Search and rescue, and searching for missing persons at the request of rescue services or families.
- Disaster response: floods, tsunamis, earthquakes, wildfires, avalanches.
- Counting and locating livestock and wildlife for conservation and agriculture.
- Research and education consistent with Section 3.
- Humanitarian missions under the oversight of independent organisations.

## 5. How the design reflects this

- **Motion only.** byeol works with motion sensors. It processes no images of people and stores no personal data.
- **Never in the flight-control loop.** Balance, stabilisation and failsafes stay with a deterministic, verified flight controller.
- **A person decides.** byeol is an aid to a human-supervised system. It does not trigger actions involving people.
- **Local processing.** byeol runs on board and needs no cloud.
- **Honest limits.** We publish where byeol fails. We report results only on synthetic data, and we say so clearly. A missed person can be a lost life, so we do not hide weaknesses.
- **Clean data.** We use only openly licensed civilian data. We excluded datasets with military funding or purpose ([docs/DESIGN_DECISIONS.md](docs/DESIGN_DECISIONS.md#d-014)), and we use no third-party trained weights.

## 6. What we do in practice

We do not provide support, consulting, priority fixes, hosting or partnership to anyone we know to be using the project in conflict with Section 3. We may publicly document known misuse.

## 7. Reporting misuse

If you know that byeol is being used against these principles, please tell us privately. Use GitHub's **"Report a vulnerability"** feature on this repository, which is private and goes only to the maintainers. Further details are in [SECURITY.md](SECURITY.md). We review every report.

## 8. Changes to this document

We may strengthen these principles at any time. We will weaken them only with a public explanation, announced at least 30 days in advance.
