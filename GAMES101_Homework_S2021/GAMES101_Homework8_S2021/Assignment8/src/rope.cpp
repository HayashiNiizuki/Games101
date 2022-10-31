#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes) {
    // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.

    //        Comment-in this part when you implement the constructor
    double x_length = (end.x - start.x) / (num_nodes - 1);
    double y_length = (end.y - start.y) / (num_nodes - 1);

    Vector2D cur = start;
    for (int i = 0; i < num_nodes; i++) {
        Mass *newNode = new Mass(cur, node_mass, false);

        if (i) {
            springs.push_back(new Spring(masses.back(), newNode, k));
        }

        masses.push_back(newNode);

        cur.x += x_length;
        cur.y += y_length;
    }
    for (auto &i : pinned_nodes) {
        masses[i]->pinned = true;
    }
}

void Rope::simulateEuler(float delta_t, Vector2D gravity) {
    for (auto &s : springs) {
        // TODO (Part 2): Use Hooke's law to calculate the force on a node
        Vector2D p = s->m1->position - s->m2->position;
        double length = p.norm();
        Vector2D f2h = -(s->k * p * (length - s->rest_length)) / length;
        s->m1->forces += f2h;
        s->m2->forces -= f2h;
    }

    for (auto &m : masses) {
        if (!m->pinned) {
            // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
            m->forces += gravity * (m->mass);

            Vector2D acceleration = m->forces / m->mass;

            m->velocity += acceleration * delta_t;
            m->position += m->velocity * delta_t;

            // TODO (Part 2): Add global damping
        }

        // Reset all forces on each mass
        m->forces = Vector2D(0, 0);
    }
}

void Rope::simulateVerlet(float delta_t, Vector2D gravity) {
    for (auto &s : springs) {
        // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
        Vector2D p = s->m1->position - s->m2->position;
        double length = p.norm();
        Vector2D f2h = -(s->k * p * (length - s->rest_length)) / length;
        s->m1->forces += f2h;
        s->m2->forces -= f2h;
    }

    for (auto &m : masses) {
        if (!m->pinned) {

            m->forces += gravity * (m->mass);
            Vector2D acceleration = m->forces / m->mass;

            Vector2D temp_position = m->position;
            Vector2D last_position = m->last_position;

            float factor = 0.00005;
            m->position = temp_position + (1 - factor) * (temp_position - last_position) + acceleration * delta_t * delta_t;
            m->last_position = temp_position;

            m->velocity += acceleration * delta_t;

            // TODO (Part 3.1): Set the new position of the rope mass

            // TODO (Part 4): Add global Verlet damping
        }

        m->forces = Vector2D(0, 0);
    }
}
} // namespace CGL
