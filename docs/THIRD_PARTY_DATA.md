# Third-party datasets and simulators: license review

**English** · [Українська](#українська)

Reviewed 2026-10-06. **No third-party dataset is included in byeol or was used to build or evaluate this release.** byeol has no learned parameters (D-001), and every published result comes from its own synthetic generator. This review is published so that others who want to validate byeol on real data can see which sources look usable, and why we excluded the others.

**Acceptance rule (D-011).** Accepted: CC0, CC BY, MIT, Apache-2.0, BSD. Rejected: NC (non-commercial), ND, SA combined with NC, non-standard wording, and "no licence stated". Also rejected: sources with military origin, funding or purpose ([ETHICS.md](../ETHICS.md)), even if their licence is permissive.

> **This is not legal advice.** Licence text was collected from primary sources through an automated web reader. Before you actually use any of these sources, open the licence page yourself and check the current terms.

## 1. Flight logs and IMU + video datasets

| Dataset | Licence (as found) | Verdict | Note |
|---|---|---|---|
| EuRoC MAV | "In Copyright – Non-Commercial Use Permitted" (ETH Research Collection) | rejected | Non-commercial |
| UZH-FPV Drone Racing | CC BY-NC-SA 3.0 | rejected | Non-commercial, share-alike |
| Mid-Air | CC BY-NC-SA 4.0 (secondary source only; site unreachable) | rejected | Non-commercial, share-alike |
| PX4 Flight Review logs | "CC-BY PX4", chosen per log by its author | rejected | Non-standard licence, per-log opt-in, geolocation, unknown origin |
| Zurich Urban MAV | "released with no restriction" (no standard licence) | rejected until clarified | Could be accepted if the authors confirm CC BY or CC0 |
| OIVIO | licence not found | rejected | — |
| INSANE, IMUFUZZER | not checked | rejected until checked | — |
| Blackbird | MIT on the repository; no licence found for the data | excluded (D-014) | Data licence unclear |
| TartanAir | CC BY 4.0 (data), BSD (code) | excluded (D-014) | Funded by a military research agency |
| TUM VI | CC BY 4.0 (data), BSD-2-Clause (code) | acceptable | Hand-held camera, not a drone. Closest usable option for checking the IMU→pixel geometry, with calibration included. |
| ALFA (CMU) | CC BY 4.0 | acceptable | Fixed-wing aircraft, no video |
| MUN-FRL | CC BY 4.0 (data files; the paper is CC BY-NC) | acceptable | Visual-inertial-LiDAR, real flights |
| TII Racing | CC BY 4.0 (data), MIT (code) | acceptable | Aggressive racing flights; unlike hovering over water |
| UAV-SEAD | CC BY 4.0 | acceptable | PX4 logs (IMU, GPS, barometer), no video |
| PinPoint validation | CC BY 4.0 (Zenodo) | acceptable | Only 2 flights |

**Conclusion.** Usable datasets exist, but none matches what byeol needs: synchronised video **over water** with IMU and height. That is why real validation requires new recordings ([RECORDING_PROTOCOL.md](RECORDING_PROTOCOL.md)).

## 2. Simulators

"Tool only" means the simulator may be run as a separate process to generate test recordings. It is never embedded in byeol, and its assets are never redistributed.

| Software | Licence (as found) | Verdict |
|---|---|---|
| PX4 SITL | BSD-3-Clause | acceptable |
| jMAVSim | BSD-3-Clause | acceptable (outdated, no camera) |
| MuJoCo | Apache-2.0 (code), CC BY 4.0 (docs) | acceptable (no ready drone with camera) |
| Gazebo Harmonic (gz-sim) | Apache-2.0; Fuel model licences vary per model | tool only |
| PX4-gazebo-models | BSD-3-Clause; individual models may have own terms | tool only |
| Gazebo Classic | Apache-2.0; end of life 2025-01 | tool only |
| AirSim / Colosseum | MIT; Unreal Engine and scenes have separate terms | tool only |
| Flightmare | MIT; Unity binaries unclear | tool only |
| ArduPilot SITL | GPL-3.0 | tool only (copyleft: separate process, never linked) |
| Isaac Sim | Apache-2.0 code; core, models and textures proprietary | tool only |
| RotorS | Apache-2.0 per file headers; no root LICENSE | tool only |
| CoppeliaSim | "CoppeliaSim Edu cannot be used commercially" | rejected |

**Not confirmed:** Mid-Air (site unreachable), OIVIO, INSANE, Gazebo Fuel model licences, the Unreal Engine EULA (page returned 403). All are treated as not accepted until someone checks them by hand.

---

## Українська

Перевірено 2026-10-06. **Жоден сторонній датасет не входить до byeol і не використовувався для створення чи оцінки цього релізу.** byeol не має навчених параметрів (D-001), а всі опубліковані результати отримано на власному синтетичному генераторі. Огляд опубліковано, щоб ті, хто хоче перевірити byeol на реальних даних, бачили, які джерела виглядають придатними і чому ми виключили решту.

**Правило прийняття (D-011).** Приймаємо CC0, CC BY, MIT, Apache-2.0, BSD. Відхиляємо NC (некомерційні), ND, SA разом із NC, нестандартні формулювання та «ліцензію не вказано». Також відхиляємо джерела з військовим походженням, фінансуванням чи призначенням ([ETHICS.uk.md](../ETHICS.uk.md)), навіть якщо ліцензія дозвільна.

> **Це не юридична консультація.** Тексти ліцензій зібрано з первинних джерел через автоматизований інструмент читання веб-сторінок. Перш ніж реально використати будь-яке з цих джерел, відкрийте сторінку ліцензії самостійно й перевірте чинні умови.

**Набори даних** (ліцензії й примітки — в англійській таблиці вище):

- **Відхилено:** EuRoC MAV, UZH-FPV, Mid-Air (некомерційні або SA); PX4 Flight Review (нестандартна ліцензія, вибір автора кожного логу, геолокація, невідоме походження); Zurich Urban MAV (немає стандартної ліцензії, можна повернути, якщо автори підтвердять CC BY або CC0); OIVIO (ліцензію не знайдено); INSANE та IMUFUZZER (не перевірено).
- **Виключено остаточно (D-014):** Blackbird (ліцензія на самі дані неясна) і TartanAir (фінансування військового дослідницького агентства).
- **Придатні:** TUM VI (ручна камера, не дрон; найближчий варіант для перевірки геометрії IMU → піксель, є калібрування), ALFA (літак, без відео), MUN-FRL (реальні польоти, візуально-інерційно-LiDAR), TII Racing (агресивні польоти, не схожі на висіння над водою), UAV-SEAD (логи PX4 без відео), PinPoint (лише 2 польоти).

**Висновок.** Придатні набори є, але жоден не дає того, що потрібно byeol: синхронізованого відео **над водою** разом з IMU й висотою. Тому для реальної перевірки потрібні нові записи ([RECORDING_PROTOCOL.uk.md](RECORDING_PROTOCOL.uk.md)).

**Симулятори.** «Лише як інструмент» означає, що симулятор можна запускати окремим процесом для генерації тестових записів, але не вбудовувати в byeol і не поширювати його ассети. Придатні: PX4 SITL, jMAVSim, MuJoCo. Лише як інструмент: Gazebo Harmonic, PX4-gazebo-models, Gazebo Classic, AirSim/Colosseum, Flightmare, ArduPilot SITL (GPL-3.0: лише окремим процесом, без лінкування), Isaac Sim, RotorS. Відхилено: CoppeliaSim (Edu-версію не можна використовувати комерційно).

**Не підтверджено:** Mid-Air (сайт недоступний), OIVIO, INSANE, ліцензії моделей Gazebo Fuel, EULA Unreal Engine (сторінка повернула 403). Усі вони вважаються неприйнятими, доки хтось не перевірить їх вручну.
