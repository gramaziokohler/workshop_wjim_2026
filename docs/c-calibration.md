<p class="step">APPENDIX</p>

# Calibration

The robot only knows its own base.
Three touches with the bit tell it where the fabrication surface is, and which way a beam lies on it.
What they give goes into `calibration`, in group **8** ([Toolpaths](07-toolpaths.md)).

## 1 · Touch three points

Two narrow beams are clamped to the surface at a right angle.
Where they meet is the origin, **A**. **B** is along one beam, **C** along the other.

<figure markdown>
![The robot touches the clamped right angle with the tip of its bit. A, B and C are measured from the robot's base frame](assets/calibration/calibration_1_measure.png)
<figcaption>The robot model with its tip at the measured A. The sizes of the clamped beams, the surface and the pedestal are illustrative.</figcaption>
</figure>

## 2 · The tip cannot reach the face

The bit has a thickness. When its wall is flush with the beam, its tip is still one radius away from the face.

<figure markdown>
![Side view: the bit's wall touches the clamped beam, its tip is one radius, 3 mm, away from the face](assets/calibration/calibration_2_radius.png)
<figcaption>Side view, bit enlarged.</figcaption>
</figure>

## 3 · Take off the radius

The offset goes only towards a beam the bit touches:
**A** sits in the corner and moves in x and y, **B** moves in x, **C** in y.

<figure markdown>
![From above: A is the tip minus (r, r, 0), B the tip minus (r, 0, 0), C the tip minus (0, r, 0)](assets/calibration/calibration_3_offsets.png)
<figcaption>From above, bit enlarged. The robot base's x points right, away from the robot; y points up.</figcaption>
</figure>

## 4 · Three points make a frame

A beam pushed into the corner is now where the software expects it.

<figure markdown>
![Plan to scale: the origin is A, X runs from A to B, Y from C to A squared up to X, Z up](assets/calibration/calibration_4_frame.png)
<figcaption>A, B and C to scale, from above, in mm in the robot's base frame. The beams are illustrative.</figcaption>
</figure>
