import java.util.*;
import java.util.concurrent.ThreadLocalRandom;

public class TSP {

    static final double EAMP = 0.1;
    static final double E_ELEC = 100.0;

    static class Node {
        int id, x, y;
        Node(int id, int x, int y) {
            this.id = id;
            this.x = x;
            this.y = y;
        }

        double distanceTo(Node other) {
            int dx = x - other.x;
            int dy = y - other.y;
            return Math.sqrt(dx * dx + dy * dy);
        }
    }

    static class Edge {
        int u, v;
        double weight;
        Edge(int u, int v, double weight) {
            this.u = u;
            this.v = v;
            this.weight = weight;
        }
    }

    static Map<Integer, Node> generateRandomNodes(int n, int maxX, int maxY) {
        Map<Integer, Node> nodes = new HashMap<>();
        Set<String> used = new HashSet<>();
        Random rand = new Random();
        int id = 1;
        while (nodes.size() < n) {
            int x = rand.nextInt(maxX + 1);
            int y = rand.nextInt(maxY + 1);
            String key = x + "," + y;
            if (!used.contains(key)) {
                nodes.put(id, new Node(id, x, y));
                used.add(key);
                id++;
            }
        }
        return nodes;
    }

    static Map<Integer, List<Edge>> buildAdjList(Map<Integer, Node> nodes, int Tr) {
        Map<Integer, List<Edge>> adj = new HashMap<>();
        for (int i : nodes.keySet()) adj.put(i, new ArrayList<>());

        for (int i : nodes.keySet()) {
            for (int j : nodes.keySet()) {
                if (i != j) {
                    double d = nodes.get(i).distanceTo(nodes.get(j));
                    if (d <= Tr) {
                        adj.get(i).add(new Edge(i, j, d));
                    }
                }
            }
        }
        return adj;
    }

    static List<List<Integer>> bfs(Map<Integer, List<Edge>> adj) {
        Set<Integer> visited = new HashSet<>();
        List<List<Integer>> components = new ArrayList<>();

        for (int start : adj.keySet()) {
            if (!visited.contains(start)) {
                List<Integer> comp = new ArrayList<>();
                Queue<Integer> q = new LinkedList<>();
                q.add(start);
                visited.add(start);

                while (!q.isEmpty()) {
                    int u = q.poll();
                    comp.add(u);
                    for (Edge e : adj.get(u)) {
                        if (!visited.contains(e.v)) {
                            visited.add(e.v);
                            q.add(e.v);
                        }
                    }
                }
                components.add(comp);
            }
        }
        return components;
    }

    static List<List<Integer>> dfs(Map<Integer, List<Edge>> adj) {
        Set<Integer> visited = new HashSet<>();
        List<List<Integer>> components = new ArrayList<>();

        for (int start : adj.keySet()) {
            if (!visited.contains(start)) {
                List<Integer> comp = new ArrayList<>();
                Stack<Integer> stack = new Stack<>();
                stack.push(start);
                visited.add(start);

                while (!stack.isEmpty()) {
                    int u = stack.pop();
                    comp.add(u);
                    for (Edge e : adj.get(u)) {
                        if (!visited.contains(e.v)) {
                            visited.add(e.v);
                            stack.push(e.v);
                        }
                    }
                }
                components.add(comp);
            }
        }
        return components;
    }

    static double dijkstra(Map<Integer, List<Edge>> adj, List<Integer> comp, int minData, int maxData, Map<String, Double> edgeWeights, int[] rendezvous) {
        Map<Integer, Integer> data = new HashMap<>();
        for (int i : comp)
            data.put(i, ThreadLocalRandom.current().nextInt(minData, maxData + 1));

        for (int u : comp) {
            for (Edge e : adj.get(u)) {
                if (comp.contains(e.v)) {
                    int d = data.get(u);
                    double energy = E_ELEC * d + EAMP * d * Math.pow(e.weight, 2) + E_ELEC * d;
                    edgeWeights.put(u + "-" + e.v, energy);
                    edgeWeights.put(e.v + "-" + u, energy);
                }
            }
        }

        int ri = comp.get(ThreadLocalRandom.current().nextInt(comp.size()));
        rendezvous[0] = ri;
        Map<Integer, Double> dist = new HashMap<>();
        for (int i : comp) dist.put(i, Double.POSITIVE_INFINITY);
        dist.put(ri, 0.0);

        PriorityQueue<Edge> pq = new PriorityQueue<>(Comparator.comparingDouble(e -> e.weight));
        pq.add(new Edge(-1, ri, 0.0));

        Set<Integer> visited = new HashSet<>();

        while (!pq.isEmpty()) {
            Edge curr = pq.poll();
            int u = curr.v;
            if (visited.contains(u)) continue;
            visited.add(u);

            for (Edge e : adj.get(u)) {
                if (comp.contains(e.v) && !visited.contains(e.v)) {
                    String key = u + "-" + e.v;
                    double energy = edgeWeights.getOrDefault(key, Double.POSITIVE_INFINITY);
                    double alt = dist.get(u) + energy;
                    if (alt < dist.get(e.v)) {
                        dist.put(e.v, alt);
                        pq.add(new Edge(u, e.v, alt));
                    }
                }
            }
        }

        double totalEnergy = 0.0;
        for (int node : comp) {
            if (node != ri && dist.containsKey(node)) {
                totalEnergy += dist.get(node);
            }
        }

        return totalEnergy;
    }

    static double mst(Map<Integer, List<Edge>> adj, List<Integer> comp, Map<String, Double> edgeWeights, int ri) {
        Set<Integer> visited = new HashSet<>();
        visited.add(ri);

        PriorityQueue<Edge> pq = new PriorityQueue<>(Comparator.comparingDouble(e -> e.weight));
        for (Edge e : adj.get(ri)) {
            if (comp.contains(e.v)) {
                String key = ri + "-" + e.v;
                pq.add(new Edge(ri, e.v, edgeWeights.getOrDefault(key, Double.POSITIVE_INFINITY)));
            }
        }

        double totalEnergy = 0.0;

        while (!pq.isEmpty()) {
            Edge edge = pq.poll();
            if (visited.contains(edge.v)) continue;
            visited.add(edge.v);
            totalEnergy += edge.weight;

            for (Edge e : adj.get(edge.v)) {
                if (comp.contains(e.v) && !visited.contains(e.v)) {
                    String key = e.u + "-" + e.v;
                    pq.add(new Edge(e.u, e.v, edgeWeights.getOrDefault(key, Double.POSITIVE_INFINITY)));
                }
            }
        }

        return totalEnergy;
    }
    
	static double greedyRobot(List<Node> rendezvousPoints) {
		Set<Node> newNodes = new HashSet<>(rendezvousPoints);
		Node curr = new Node(-1, 0, 0);
		double totalEnergy = 0;

		while (!newNodes.isEmpty()) {
			Node next = null;
			double minEdge = Double.MAX_VALUE;

			for (Node tryNode : newNodes) {
				double edge = curr.distanceTo(tryNode);
				if (edge < minEdge) {
					minEdge = edge;
					next = tryNode;
				}
			}

			totalEnergy += minEdge * 100;
			curr = next;
			newNodes.remove(next);
		}

		totalEnergy += curr.distanceTo(new Node(-1, 0, 0)) * 100;
		return totalEnergy;
	}

	static double approxRobot(List<Node> rendezvousPoints) {
		int n = rendezvousPoints.size();
		double[][] edge = new double[n][n];

		for (int i = 0; i < n; i++) {
			Node a = rendezvousPoints.get(i);
			for (int j = 0; j < n; j++) {
				Node b = rendezvousPoints.get(j);
				edge[i][j] = a.distanceTo(b);
			}
		}

		boolean[] visited = new boolean[n];
		double[] key = new double[n];
		int[] parent = new int[n];
		Arrays.fill(key, Double.MAX_VALUE);
		key[0] = 0;

		for (int count = 0; count < n; count++) {
			int u = -1;
			for (int i = 0; i < n; i++) {
				if (!visited[i] && (u == -1 || key[i] < key[u])) u = i;
			}
			visited[u] = true;

			for (int v = 0; v < n; v++) {
				if (!visited[v] && edge[u][v] < key[v]) {
					key[v] = edge[u][v];
					parent[v] = u;
				}
			}
		}

		Map<Integer, List<Integer>> tree = new HashMap<>();
		for (int i = 1; i < n; i++) {
			tree.computeIfAbsent(parent[i], k -> new ArrayList<>()).add(i);
		}

		List<Integer> tour = new ArrayList<>();
		preorder(0, tree, tour);

		List<Node> fullTour = new ArrayList<>();
		Set<Integer> seen = new HashSet<>();

		fullTour.add(new Node(-1, 0, 0));
		for (int i : tour) {
			if (!seen.contains(i)) {
				fullTour.add(rendezvousPoints.get(i));
				seen.add(i);
			}
		}
		fullTour.add(new Node(-1, 0, 0));


		double totalEnergy = 0.0;
		for (int i = 0; i < fullTour.size() - 1; i++) {
			totalEnergy += fullTour.get(i).distanceTo(fullTour.get(i + 1)) * 100;
		}

		return totalEnergy;
	}

	static void preorder(int u, Map<Integer, List<Integer>> tree, List<Integer> tour) {
		tour.add(u);
		for (int v : tree.getOrDefault(u, new ArrayList<>())) {
			preorder(v, tree, tour);
		}
	}


    public static void main(String[] args) {
        Scanner sc = new Scanner(System.in);
        System.out.print("Enter the number of sensor nodes: ");
        int n = sc.nextInt();
        System.out.print("Enter the x axis: ");
        int x = sc.nextInt();
        System.out.print("Enter the y axis: ");
        int y = sc.nextInt();

        Map<Integer, Node> nodes = generateRandomNodes(n, x, y);

        System.out.print("Enter the transmission range: ");
        int Tr = sc.nextInt();
        sc.nextLine();

        System.out.print("Enter a for adjacency matrix or b for adjacency list: ");
        String graphType = sc.nextLine();

        System.out.print("Enter a for BFS or b for DFS: ");
        String traversal = sc.nextLine();

        System.out.print("Enter minimum and maximum SEPARATED BY A SPACE for data range: ");
        int minData = sc.nextInt(), maxData = sc.nextInt();

        Map<Integer, List<Edge>> adjList = buildAdjList(nodes, Tr);
        List<List<Integer>> components;
        if (traversal.equals("a")) {
            System.out.println("\nBFS List Connectivity");
            components = bfs(adjList);
        } else {
            System.out.println("\nDFS List Connectivity");
            components = dfs(adjList);
        }

        System.out.println("There are " + components.size() + " connected components\n");
        for (List<Integer> comp : components) {
            System.out.println(comp);
        }
        List<Node> rendezvousPoints = new ArrayList<>();
        double totalEnergy = 0.0, totalMSTEnergy = 0.0;
        System.out.println("\nShortest Path Energy Consumption");
        for (List<Integer> comp : components) {
            if (comp.size() <= 1) continue;
            Map<String, Double> edgeWeights = new HashMap<>();
            int[] rendezvous = new int[1];
            double spEnergy = dijkstra(adjList, comp, minData, maxData, edgeWeights, rendezvous);
            rendezvousPoints.add(nodes.get(rendezvous[0]));
            totalEnergy += spEnergy;
            System.out.printf("\nRendezvous point: %d\nEnergy Consumption: %.2f\n", rendezvous[0], spEnergy);
            double mstEnergy = mst(adjList, comp, edgeWeights, rendezvous[0]);
            totalMSTEnergy += mstEnergy;
            System.out.printf("MST Energy: %.2f\n", mstEnergy);
        }

        System.out.printf("\nShortest Path Total Network Energy Consumption: %.2f\n", totalEnergy);
        System.out.printf("MST Total Network Energy Consumption: %.2f\n", totalMSTEnergy);

        System.out.println("\nRobot Energy:");
        double greedy = greedyRobot(rendezvousPoints);
        System.out.printf("Greedy Robot Energy: %.2f J\n", greedy);
        double approx = approxRobot(rendezvousPoints);
        System.out.printf("2 Approximate Robot Energy: %.2f J\n", approx);

    }
}
