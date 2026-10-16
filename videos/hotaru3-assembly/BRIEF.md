---
workflow: general-video
flow: automation
storyboard: no
message: "Every part and every M3 bolt and nut, in the order that actually fits, with the hand cuts the printed parts need."
destination: youtube
aspect: 1920x1080
language: en
length: about 3.5 min
audience: "the builder: tub, tower and arm already printed, servos and a bag of M3 on the desk"
---

## Intent

The user asked for "an assembly video, 3d animation of insertions, everything
end to end, what all parts to put and where to put m3 heat inserts". HOTARU 3
has no heat-set inserts: every bolt either runs into a hex nut sitting in a
printed pocket or slot, or cuts its own thread in a 2.6 mm pilot hole (pan
servo into the tub's pillars, lip ring into the tub's cone), or goes into a
servo's tapped output shaft. The video shows that, one bolt and one nut at a
time.

## Customizations

- The four hand cuts on already-printed parts are shown as steps: speaker
  rails snipped to 10 mm, the mic slot's back end wall nibbled out, a groove
  for the pan servo's cable boot (only if it catches), and the bar over the
  tower's servo window sawn out and put back as a keeper. The user will not
  reprint anything.
- A running tally of bolts and nuts used, against the 48 bolts / 35 nuts total.

## Notes

- Geometry is the project's own SolidWorks CAD (sw/h3_video.py): no mock-ups.
- Order and approach paths are sw/h3_insert.py's, which SolidWorks checks for
  interference at every 2 mm of travel. Bench work is shown beside the lamp
  and carried over in free air.
- Inferred, not asked (the user wants it built, not discussed): destination,
  aspect and language from the v2 video's brief; length from the step count.
