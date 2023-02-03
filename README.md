# NPC-Navigation
navigation system for NPCs in a 3D game using pathfinding algorithms like A* &amp; Dijkstra's

<!-- Improved compatibility of back to top link: See: https://github.com/othneildrew/Best-README-Template/pull/73 -->
<a name="readme-top"></a>
<!--
*** Thanks for checking out the Best-README-Template. If you have a suggestion
*** that would make this better, please fork the repo and create a pull request
*** or simply open an issue with the tag "enhancement".
*** Don't forget to give the project a star!
*** Thanks again! Now go create something AMAZING! :D
-->

<!-- PROJECT SHIELDS -->

<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->
<p align="center">
    <a href="https://github.com/GlitchyMako/NPC-Navigation/releases">
        <img alt="GitHub release (latest by semver)" src="https://img.shields.io/github/v/release/devicons/devicon?color=%2360be86&label=Latest%20release&style=for-the-badge&sort=semver">
    </a>
    <a href="https://github.com/GlitchyMako/NPC-Navigation/graphs/contributors">
        <img alt="GitHub contributors" src="https://img.shields.io/github/contributors-anon/devicons/devicon?color=%2360be86&style=for-the-badge">
    </a>
    <a href="https://github.com/GlitchyMako/NPC-Navigation/actions">
        <img alt="GitHub branch checks state" src="https://img.shields.io/github/checks-status/devicons/devicon/master?color=%2360be86&style=for-the-badge">
    </a>
    <a href="https://github.com/GlitchyMako/NPC-Navigation/issues?q=is%3Aopen+is%3Aissue+label%3Arequest%3Aicon">
        <img alt="GitHub issues by-label" src="https://img.shields.io/github/issues/devicons/devicon/request:icon?color=%2360be86&label=icon%20requests&style=for-the-badge">
    </a>
    <a href="https://github.com/GlitchyMako/NPC-Navigation/stargazers">
        <img alt="GitHub Repo stars" src="https://img.shields.io/github/stars/devicons/devicon?color=%2360be86&label=github%20stars&style=for-the-badge">
    </a>
</p>

<!-- PROJECT LOGO -->

<br />
<div align="center">

<h1 align="center">NPC Navigation</h1>

  <p align="center">
    <br />
    <a href="https://github.com/github_username/repo_name"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://github.com/github_username/repo_name">View Demo</a>
    ·
    <a href="https://github.com/github_username/repo_name/issues">Report Bug</a>
    ·
    <a href="https://github.com/github_username/repo_name/issues">Request Feature</a>
  </p>
</div>



<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

This code implements a navigation system for non-player characters (NPCs) in a 3D RPG game using pathfinding algorithms like A* or Dijkstra's. The code allows NPCs to find the shortest path from their current location to a target location, taking into account obstacles in the game world.

## Algorithm

The NPC Navigation code uses the A* algorithm to find the shortest path from the starting point to the target. A* is a popular pathfinding algorithm that balances the cost of the path with an estimate of the remaining distance to the target, allowing it to find an optimal path more quickly than Dijkstra's algorithm.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Built With

<a href="https://www.python.org" target="_blank"> <img src="https://raw.githubusercontent.com/devicons/devicon/master/icons/python/python-original.svg" alt="python" width="40" height="40"/> </a>

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- GETTING STARTED -->
## Getting Started

This is an example of how you may give instructions on setting up your project locally.
To get a local copy up and running follow these simple example steps.

<!-- USAGE EXAMPLES -->
## Usage

To use the NPC Navigation code in your 3D RPG game, you'll need to provide a representation of your game world as a graph, with nodes representing locations and edges representing paths between those locations. The code uses the graph to search for the shortest path from the starting point to the target.

Here's a code snippet showing how to use the NPC Navigation code:
```python
from npc_navigation import NPCNavigation

navigation = NPCNavigation()

# Define the graph representation of your game world
graph = {
    (0, 0, 0): [(0, 1, 0), (1, 0, 0)],
    (0, 1, 0): [(0, 0, 0), (1, 1, 0)],
    (1, 0, 0): [(0, 0, 0), (1, 1, 0)],
    (1, 1, 0): [(0, 1, 0), (1, 0, 0)]
}

# Find the shortest path from the starting point to the target
start = (0, 0, 0)
target = (1, 1, 0)
path = navigation.find_path(graph, start, target)

print(path)  # [(0, 0, 0), (1, 0, 0), (1, 1, 0)]
```

```python
graph = GridWithWeights(10, 10)
start, goal = (1, 4), (8, 5)

navigator = NPCNavigator(graph, start, goal)
came_from, cost_so_far = navigator.search()

path = reconstruct_path(came_from, start, goal)
print("Path: ", path)
print("Cost: ", cost_so_far[goal])
```
In the above example, we first create a GridWithWeights object with a grid of 10 by 10, then we define the start and goal points. Next, we create an instance of the NPCNavigator class, passing the graph, start, and goal as arguments. Finally, we call the search method on the navigator object, which returns two dictionaries, came_from and cost_so_far.

The reconstruct_path function can be used to construct the path from the start to the goal based on the came_from dictionary. The final cost of reaching the goal can be obtained from the cost_so_far dictionary.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Cost Function

The cost function calculates the cost of moving from one node to another in the graph. The code uses the manhattan_distance function to calculate the cost, which is a simple measure of the distance between two points in a grid.

Here's a code snippet showing the implementation of the cost function:
```python
def cost(self, current, next):
    return manhattan_distance(current, next)
```
<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- Customization -->
## Customization

The NPC Navigation code can be customized to fit the specific needs of your 3D RPG game. For example, you may want to use a different cost function that takes into account additional factors such as the difficulty of traversing certain terrain.

To customize the code, simply create a new class that inherits from the NPCNavigation class and override the methods you want to change.

<!-- Performance -->
## Performance

The performance of the NPC Navigation code will depend on the size and complexity of your game world and the performance of your computer. In general, the code will perform well for small to medium-sized game worlds, but for larger game worlds, you may need to consider using a more optimized algorithm or breaking the game world into smaller regions to improve performance.

<!-- Limitations -->
## Limitations

The code assumes that the game world is grid-based and that movements can only be made in the four cardinal directions (up, down, left, and right). Additionally, the code assumes that the cost of moving from one tile to another is the same in all directions. These limitations may need to be addressed if the code is used in a game world with more complex movement rules.

<!-- Technical Implementation -->
## Technical Implementation

```python
class AStar:
    def __init__(self, graph, start, end):
        self.graph = graph
        self.start = start
        self.end = end
        self.open_list = []
        self.closed_list = []
        self.cost_so_far = {}
        self.came_from = {}
        self.heuristic = {}

    def search(self):
        heapq.heappush(self.open_list, (0, self.start))
        self.cost_so_far[self.start] = 0
        self.heuristic[self.start] = manhattan_distance(self.start, self.end)

        while self.open_list:
            current = heapq.heappop(self.open_list)[1]

            if current == self.end:
                return self.reconstruct_path()

            self.closed_list.append(current)

            for next in self.graph.neighbors(current):
                new_cost = self.cost_so_far[current] + self.cost(current, next)

                if next in self.closed_list and new_cost >= self.cost_so_far[next]:
                    continue

                if next not in self.open_list or new_cost < self.cost_so_far[next]:
                    self.cost_so_far[next] = new_cost
                    priority = new_cost + self.heuristic[next]
                    heapq.heappush(self.open_list, (priority, next))
                    self.came_from[next] = current

    def reconstruct_path(self):
        path = []
        current = self.end
        while current != self.start:
            path.append(current)
            current = self.came_from[current]
        path.append(self.start)
        path.reverse()
        return path

    def cost(self, current, next):
        return manhattan_distance(current, next)


class Dijkstra:
    def __init__(self, graph, start, end):
        self.graph = graph
        self.start = start
        self.end = end
        self.open_list = []
        self.closed_list = []
        self.cost_so_far = {}
        self.came_from = {}

    def search(self):
        heapq.heappush(self.frontier, (0, self.start))
		self.came_from[self.start] = None
		self.cost_so_far[self.start] = 0
        
        while len(self.frontier) > 0:
        current = heapq.heappop(self.frontier)[1]

        if current == self.goal:
            break

        for next in self.graph.neighbors(current):
            new_cost = self.cost_so_far[current] + self.cost(current, next)
            if next not in self.cost_so_far or new_cost < self.cost_so_far[next]:
                self.cost_so_far[next] = new_cost
                priority = new_cost + self.heuristic(next, self.goal)
                heapq.heappush(self.frontier, (priority, next))
                self.came_from[next] = current

    return self.came_from, self.cost_so_far
```

The NPC Navigation code is written in Python and makes use of a graph data structure to represent the game world. The nodes in the graph represent the different locations in the game world, and the edges represent the paths between these locations.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- CONCLUSION -->
## Conclusion

The NPC Navigation code uses the A* algorithm to implement pathfinding for non-player characters (NPCs) in a game. It allows NPCs to find the shortest path from a start point to a goal point while taking into account the cost of moving between nodes in the game world. While this implementation has some limitations, it provides a good starting point for more complex pathfinding solutions in a game.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- Future Work -->
## Future Work

Potential future work for the NPC Navigation code includes adding support for diagonal movements, incorporating additional factors such as terrain type, elevation, or obstacles, and optimizing the code for performance. Ongoing maintenance and support will also be necessary to ensure that the code continues to work as intended.

<!-- CONTACT -->
## Contact

Tomás Garrett - [@LinkedIn](https://www.linkedin.com/in/tomás-almeida-garrett-83461a183/)

Project Link: [https://github.com/glitchyMako/Face-detection](https://github.com/GlitchyMako/Face-detection)

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/github_username/repo_name.svg?style=for-the-badge
[contributors-url]: https://github.com/GlitchyMako/Face-detection/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/github_username/repo_name.svg?style=for-the-badge
[forks-url]: https://github.com/GlitchyMako/Face-detection/network/members
[stars-shield]: https://img.shields.io/github/stars/github_username/repo_name.svg?style=for-the-badge
[stars-url]: https://github.com/GlitchyMako/Face-detection/stargazers
[issues-shield]: https://img.shields.io/github/issues/github_username/repo_name.svg?style=for-the-badge
[issues-url]: https://github.com/GlitchyMako/Face-detection/issues
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://www.linkedin.com/in/tomás-almeida-garrett-83461a183/
