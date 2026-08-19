/**
 * Drawing-plane heights — single source of truth for the two operating
 * planes of the drawing pipeline.
 *
 * The pen plotter works on two explicit planes:
 *
 *  - DRAW_PLANE_Z    (writing plane): the Z height at which the marker
 *                    touches the paper. This is the z=const plane the IK
 *                    solves for while drawing.
 *  - TRAVEL_PLANE_Z  (transport plane): the Z height used for pen-up moves,
 *                    defined as DRAW_PLANE_Z + TRAVEL_LIFT_MM so the marker
 *                    clears the paper by a fixed lift that absorbs gravity
 *                    sag and servo backlash.
 *
 * The G-code parser maps every pen-down move to DRAW_PLANE_Z and every
 * pen-up move to TRAVEL_PLANE_Z (the file's own Z words are inert). Keeping
 * the lift as an explicit constant — instead of scattered `+5` defaults —
 * makes the clearance a single, tunable knob.
 *
 * Module must stay free of the wasm import chain: `cipra/loadGcodeText.ts`
 * imports these constants to keep that module wasm-free.
 */

/** Writing plane: marker touches the paper at this Z (mm). */
export const DRAW_PLANE_Z = 80;

/**
 * Marker lift above the writing plane for pen-up travel (mm).
 * The 5-point test proved a 5 mm nominal lift is NOT enough: gravity sag +
 * servo backlash keep the marker touching the paper. 40 mm gives a safe
 * effective clearance (the v2 experiment validated 60 mm; if the marker
 * still drags after mechanical backlash work, raise this single value).
 */
export const TRAVEL_LIFT_MM = 40;

/** Transport plane: pen-up moves run at this Z (mm). */
export const TRAVEL_PLANE_Z = DRAW_PLANE_Z + TRAVEL_LIFT_MM;
