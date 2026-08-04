# Visible clipping correction

The prior practical-tabletop renderer added a decorative actuator housing, guide rails, ram, support cap, fasteners, and nearby tabletop props that did not exist in the rigid-body collision world. Dice were therefore allowed to occupy the same volume as those render-only objects.

This correction:

- removes every render-only solid from the dice scatter envelope;
- recesses actuator and guide geometry below the analytic tabletop;
- renders the moving support leaf at exactly its serialized collidable dimensions;
- removes the 2.4 mm decorative cap that intersected the bottom course;
- removes the fixed central post visible inside the final pile.

The rigid-body cache and dice trajectories are otherwise unchanged. The cache still reports a maximum buried die-to-die residual overlap of about 1.05 mm in the late pile; that is a separate contact-solver limitation from the clearly visible actuator clipping corrected here.
