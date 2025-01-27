### Boids Algorythm Visualization in MATLAB ###
This application, allows you to manipulate various variables to simulate a flock of boids moving around, trying to avoid each other, borders and predators.
Boid is represented as a rectangle, where as predator is represented as a triangle.

![boidsAlgorythmApplicationLook](https://github.com/user-attachments/assets/aa33e474-8aff-48fe-a842-9bebb1adaac0)


# TODO:
- improve performance, try using multiple cores to calculate each boid/predator seperately (will probably require additional struct of boids and predators)
- use different plane, other than built in axes (I ve read that there are faster alternatives)
- improve boids and predators movement or add additional varaibles to change
- allow predators to "eat" boids and with time a new boid should appear. Also if predator does not eat a boid in some period of time, it should die? A cool idea.
