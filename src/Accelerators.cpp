#include "Primitive.h"
#include "Threading.hpp"
#include <cstring>
#include <algorithm>

struct OctTree : IntersectionAccelerator {
	struct Node {
		BBox box;
		Node *children[8] = {nullptr, };
		std::vector<Intersectable*> primitives;
		bool isLeaf() const {
			return children[0] == nullptr;
		}
	};

	std::vector<Intersectable*> allPrimitives;
	Node *root = nullptr;
	int depth = 0;
	int leafSize = 0;
	int nodes = 0;
	int MAX_DEPTH = 35;
	int MIN_PRIMITIVES = 10;

	void clear(Node *n) {
		if (!n) {
			return;
		}

		for (int c = 0; c < 8; c++) {
			clear(n->children[c]);
			delete n->children[c];
		}
	}

	void clear() {
		clear(root);
		allPrimitives.clear();
	}

	void addPrimitive(Intersectable* prim) override {
		allPrimitives.push_back(prim);
	}

	void build(Node *n, int currentDepth = 0) {
		if (currentDepth >= MAX_DEPTH || n->primitives.size() <= MIN_PRIMITIVES) {
			leafSize = std::max(leafSize, int(n->primitives.size()));
			return;
		}

		depth = std::max(depth, currentDepth);
		BBox childBoxes[8];
		n->box.octSplit(childBoxes);

		for (int c = 0; c < 8; c++) {
			Node *& child = n->children[c];
			child = new Node;
			nodes++;
			memset(child->children, 0, sizeof(child->children));
			child->box = childBoxes[c];
			for (int r = 0; r < n->primitives.size(); r++) {
				if (n->primitives[r]->boxIntersect(child->box)) {
					child->primitives.push_back(n->primitives[r]);
				}
			}
			if (child->primitives.size() == n->primitives.size()) {
				build(child, MAX_DEPTH + 1);
			} else {
				build(child, currentDepth + 1);
			}
		}
		n->primitives.clear();
	}

	void build(Purpose purpose) override {
		const char *treePurpose = "";
		if (purpose == Purpose::Instances) {
			MAX_DEPTH = 5;
			MIN_PRIMITIVES = 4;
			treePurpose = " instances";
		} else if (purpose == Purpose::Mesh) {
			MAX_DEPTH = 35;
			MIN_PRIMITIVES = 20;
			treePurpose = " mesh";
		}

		if (root) {
			clear(root);
			delete root;
		}

		printf("Building%s oct tree with %d primitives... ", treePurpose, int(allPrimitives.size()));
		Timer timer;
		nodes = leafSize = depth = 0;
		root = new Node();
		root->primitives.swap(allPrimitives);
		for (int c = 0; c < root->primitives.size(); c++) {
			root->primitives[c]->expandBox(root->box);
		}
		build(root);
		//printf(" done in %lldms, nodes %d, depth %d, %d leaf size\n", timer.toMs(timer.elapsedNs()), nodes, depth, leafSize);
		printf(" done in %lldms, nodes %d, depth %d, %d leaf size\n", (long long)timer.toMs(timer.elapsedNs()), nodes, depth, leafSize);

	}

	bool intersect(Node *n, const Ray& ray, float tMin, float &tMax, Intersection& intersection) {
		bool hasHit = false;

		if (n->isLeaf()) {
			for (int c = 0; c < n->primitives.size(); c++) {
				if (n->primitives[c]->intersect(ray, tMin, tMax, intersection)) {
					tMax = intersection.t;
					hasHit = true;
				}
			}
		} else {
			for (int c = 0; c < 8; c++) {
				if (n->children[c]->box.testIntersect(ray)) {
					if (intersect(n->children[c], ray, tMin, tMax, intersection)) {
						tMax = intersection.t;
						hasHit = true;
					}
				}
			}
		}

		return hasHit;
	}

	bool intersect(const Ray& ray, float tMin, float tMax, Intersection& intersection) override {
		return intersect(root, ray, tMin, tMax, intersection);
	}

	bool isBuilt() const override {
		return root != nullptr;
	}

	~OctTree() override {
		clear();
	}
};

struct BVHTree : IntersectionAccelerator {
    // BVHNode represents a node in the BVH tree.
    struct BVHNode {
        BBox box; // Axis-aligned bounding box for this node.
        BVHNode *left = nullptr; // Pointer to the left child node.
        BVHNode *right = nullptr; // Pointer to the right child node.
        int start = 0, end = 0; // Range of primitives for leaf nodes.

        // Determines if the node is a leaf (no children).
        bool isLeaf() const {
            return left == nullptr && right == nullptr;
        }
    };

    // All primitives managed by this BVH tree.
    std::vector<Intersectable*> allPrimitives;
    BVHNode *root = nullptr; // Root node of the BVH tree.
    int MAX_DEPTH = 35; // Maximum recursion depth for tree building.
    int MIN_PRIMITIVES = 10; // Minimum primitives in a leaf node.

    // Recursively deletes a node and its children.
    void clearNode(BVHNode *n) {
        if (!n) return; // Base case: null node.
        clearNode(n->left); // Recursively delete left subtree.
        clearNode(n->right); // Recursively delete right subtree.
        delete n; // Delete the current node.
    }

    // Clears the entire tree structure and resets the root.
    void clear() override {
        clearNode(root); // Delete all nodes starting from the root.
        root = nullptr; // Reset root to null.
        allPrimitives.clear(); // Clear all primitives.
    }

    // Adds a primitive to the list of managed primitives.
    void addPrimitive(Intersectable *prim) override {
        allPrimitives.push_back(prim);
    }

    // Structure to store a primitive and its centroid for sorting.
    struct PrimRef {
        Intersectable* prim; // Pointer to the primitive.
        vec3 centroid; // Centroid of the primitive's bounding box.
    };

    // Computes the bounds of a range of primitives.
    void computeBounds(int start, int end, BBox &bounds, BBox &centroidBounds, std::vector<PrimRef> &refs) {
        bounds = BBox(); // Initialize empty bounding box.
        centroidBounds = BBox(); // Initialize empty centroid bounding box.
        for (int i = start; i < end; i++) {
            BBox tmpBox;
            refs[i].prim->expandBox(tmpBox); // Expand primitive's bounding box.
            bounds.add(tmpBox); // Add to overall bounding box.
            centroidBounds.add(refs[i].centroid); // Add centroid to centroid bounds.
        }
    }

    // Recursively builds the BVH tree.
    BVHNode* buildRecursive(int start, int end, std::vector<PrimRef> &refs, int depth) {
        BVHNode *node = new BVHNode(); // Create a new node.

        BBox bounds, centroidBounds;
        computeBounds(start, end, bounds, centroidBounds, refs); // Compute bounds for the range.
        node->box = bounds; // Set the node's bounding box.

        int nPrims = end - start; // Number of primitives in this range.
        if (depth >= MAX_DEPTH || nPrims <= MIN_PRIMITIVES) {
            // Create a leaf node if depth or primitive count thresholds are met.
            node->start = start;
            node->end = end;
            return node;
        }

        // Determine the axis to split along (largest dimension of centroid box).
        vec3 diff = centroidBounds.max - centroidBounds.min;
        int axis = 0;
        if (diff.y > diff.x && diff.y > diff.z) axis = 1;
        else if (diff.z > diff.x && diff.z > diff.y) axis = 2;

        // Midpoint of the centroids along the chosen axis.
        float mid = 0.5f * (centroidBounds.min[axis] + centroidBounds.max[axis]);

        // Partition primitives by midpoint.
        int midIndex = std::partition(refs.begin() + start, refs.begin() + end, [axis, mid](const PrimRef &p) {
            return p.centroid[axis] < mid;
        }) - refs.begin();

        // Force a split if all primitives are on one side.
        if (midIndex == start || midIndex == end) {
            midIndex = (start + end) / 2;
        }

        // Recursively build child nodes.
        node->left = buildRecursive(start, midIndex, refs, depth + 1);
        node->right = buildRecursive(midIndex, end, refs, depth + 1);

        return node;
    }

    // Builds the BVH tree based on the specified purpose.
    void build(Purpose purpose) override {
        if (root) {
            clear(); // Clear existing tree if present.
        }

        const char *treePurpose = "";
        if (purpose == Purpose::Instances) {
            MAX_DEPTH = 5; // Optimize for instances.
            MIN_PRIMITIVES = 4;
            treePurpose = " instances";
        } else if (purpose == Purpose::Mesh) {
            MAX_DEPTH = 35; // Optimize for meshes.
            MIN_PRIMITIVES = 20;
            treePurpose = " mesh";
        }

        printf("Building%s BVH with %d primitives... ", treePurpose, (int)allPrimitives.size());
        Timer timer;

        // Create references for all primitives, including their centroids.
        std::vector<PrimRef> refs(allPrimitives.size());
        for (int i = 0; i < (int)allPrimitives.size(); i++) {
            BBox tmpBox;
            allPrimitives[i]->expandBox(tmpBox); // Get bounding box for the primitive.
            vec3 c = (tmpBox.min + tmpBox.max) * 0.5f; // Compute centroid.
            refs[i].prim = allPrimitives[i];
            refs[i].centroid = c;
        }

        root = buildRecursive(0, (int)refs.size(), refs, 0); // Start recursive building.

        // Rearrange allPrimitives according to refs.
        std::vector<Intersectable*> sortedPrims(allPrimitives.size());
        for (int i = 0; i < (int)refs.size(); i++) {
            sortedPrims[i] = refs[i].prim;
        }
        allPrimitives.swap(sortedPrims);

		printf(" done in %lldms\n", static_cast<long long>(timer.toMs(timer.elapsedNs())));
    }

    // Recursively intersects a ray with the BVH nodes.
    bool intersectNode(BVHNode *node, const Ray& ray, float tMin, float &tMax, Intersection &intersection) {
        if (!node) return false; // Base case: null node.

        // Test intersection with the node's bounding box.
        if (!node->box.testIntersect(ray)) {
            return false;
        }

        bool hit = false;
        if (node->isLeaf()) {
            // Test intersection with primitives in the leaf node.
            for (int i = node->start; i < node->end; i++) {
                if (allPrimitives[i]->intersect(ray, tMin, tMax, intersection)) {
                    tMax = intersection.t; // Update closest intersection.
                    hit = true;
                }
            }
        } else {
            // Recursively check child nodes.
            bool hitLeft = intersectNode(node->left, ray, tMin, tMax, intersection);
            bool hitRight = intersectNode(node->right, ray, tMin, tMax, intersection);
            hit = hitLeft || hitRight;
        }

        return hit;
    }

    // Public interface to test intersection with the BVH tree.
    bool intersect(const Ray& ray, float tMin, float tMax, Intersection &intersection) override {
        if (!root) return false; // No tree built.
        return intersectNode(root, ray, tMin, tMax, intersection);
    }

    // Checks if the BVH tree has been built.
    bool isBuilt() const override {
        return root != nullptr;
    }

    // Destructor to clean up resources.
    ~BVHTree() override {
        clear(); // Ensure all resources are released.
    }
};

// Factory function to create a default accelerator.
AcceleratorPtr makeDefaultAccelerator() {
    return AcceleratorPtr(new BVHTree());
    // Alternative implementations:
    // return AcceleratorPtr(new KDTree());
    // return AcceleratorPtr(new OctTree());
}


