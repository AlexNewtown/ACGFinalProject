HOMEWORK 3: RAY TRACING, RADIOSITY, & PHOTON MAPPING

NAME:  Nathaniel Wheeler



ESTIMATE OF # OF HOURS SPENT ON THIS ASSIGNMENT:  20



COLLABORATORS AND OTHER RESOURCES: List the names of everyone you
talked to about this assignment and all of the resources (books,
online reference material, etc.) you consulted in completing this
assignment.

RAYTRACING:
I implemented stratified ray tracing, which improves convergence significantly, but at almost no extra cost. No bugs that I can think of.



RADIOSITY:
I've found no bugs in my implementation so far, but I have noticed the huge performance hit when using shadow samples to test for occluders. The amount of time it takes to compute form factors when using shadows increases exponentially when the surfaces are subdivided. One thing to mention is the image I get when doing the sphere without shadows is the whole image looks darker than the one on the website. I think this is because I don't have support for an ambient term; given shadows cause the form factors to give a higher percentage of light to unoccluded areas, I think that ambient term is all that it would take to fix it.



PHOTON MAPPING:
I have the photon mapping part done, and have a buggy implementation for gathering. However the gathering is very slow, and has to use a multiplier to show up right. Also, even though I check to make sure photons are in the sphere, I'm still getting artifacts from the KD-tree in my final render. On the bright side, I am confident my photon mapping itself is correct; when run with lines given in the gradesheet, the layout of my render matches that on the website, including the accents on the caustic. If I could spend more time on this, I would try to find a more efficient way to do photon gathering.



OTHER NEW FEATURES OR EXTENSIONS FOR EXTRA CREDIT:
Include instructions for use and test cases and sample output as appropriate.
