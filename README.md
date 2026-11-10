# Arriel 1D1 Engine Lab

An interactive cutaway and teaching simulator of the Safran (Turbomeca) Arriel 1D1 free-turbine turboshaft, the engine of the AS350 B2.

**Live site:** https://myquite.github.io/arriel-1d1-engine-lab/

Start the engine, fly it, and inject faults. The page shows:

- An animated half-section with airflow coloured by gas temperature, fuel spray, flame and rotating parts. Click the 11 numbered parts for notes.
- Cockpit gauges (N1, N2/NR, TOT, torque, oil pressure and temperature) with bands from the certified limits, warning lamps and an exceedance log.
- A live temperature–entropy (Brayton cycle) diagram with station pressures and temperatures.
- Where the fuel energy goes, and how turbine work splits between the compressors and the rotor.
- A self-checking lab sheet: governing, hot-and-high power loss, normal start, hot start abort, flameout and autorotation.

## Files

| File | Purpose |
| --- | --- |
| `index.html` | Page markup |
| `css/styles.css` | Styles, with light and dark themes |
| `js/app.js` | Engine model, rendering and UI |

No build step. Open `index.html` in a browser, or serve the folder with any static server (for example `python3 -m http.server`).

## Data and model

Ratings and limits come from the [EASA Type-Certificate Data Sheet E.073, Issue 05 (20 February 2025)](https://www.easa.europa.eu/en/downloads/16994/en). Behaviour between those limits is a simplified teaching model: mass flow, pressure ratio, efficiencies, rotor load and the start sequence are approximations, listed on the page.

## Disclaimer

For learning only. This is not an approved training device and not a substitute for the engine manuals or the helicopter flight manual. Arriel is a trademark of Safran Helicopter Engines. This project is not affiliated with Safran.
