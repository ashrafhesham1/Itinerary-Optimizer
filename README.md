# Itinerary Optimizer

## Table of contents

- [Description](#description)
- [How does it work](#how-does-it-work)
- [Technologies](#technologies)

## Description

The Itinerary Optimizer is a tool designed to generate optimized itineraries for visiting a given set of locations within a specified number of days by approaching the problem as a traveling vehicles routing problem.

## How does it work

The optimizer is able to generate optimized itinerary by approaching the problem as traveling vehicles routing problem and using **OR-Tools** to implement an algorithm that generates optimized itineraries. The algorithm startins with a **greedy approach** in order to construct an initial solution then uses the **Local Search algorithm** to optimize it in order to approach the optimal solution.

## Technologies

or-tools
