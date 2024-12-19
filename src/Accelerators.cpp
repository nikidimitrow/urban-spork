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

/// TODO: Implement one/both or any other acceleration structure and change makeDefaultAccelerator to create it
struct KDTree : IntersectionAccelerator {
	void addPrimitive(Intersectable *prim) override {}
	void clear() override {}
	void build(Purpose purpose) override {}
	bool isBuilt() const override { return false; }
	bool intersect(const Ray &ray, float tMin, float tMax, Intersection &intersection) override { return false; }
};


struct BVHTree : IntersectionAccelerator {
    struct BVHNode {
        BBox box;
        BVHNode *left = nullptr;
        BVHNode *right = nullptr;
        int start = 0, end = 0; // indices into allPrimitives for leaf nodes

        bool isLeaf() const {
            return left == nullptr && right == nullptr;
        }
    };

    std::vector<Intersectable*> allPrimitives;
    BVHNode *root = nullptr;
    int MAX_DEPTH = 35;
    int MIN_PRIMITIVES = 10;

    void clearNode(BVHNode *n) {
        if (!n) return;
        clearNode(n->left);
        clearNode(n->right);
        delete n;
    }

    void clear() override {
        clearNode(root);
        root = nullptr;
        allPrimitives.clear();
    }

    void addPrimitive(Intersectable *prim) override {
        allPrimitives.push_back(prim);
    }

    struct PrimRef {
        Intersectable* prim;
        vec3 centroid;
    };

    void computeBounds(int start, int end, BBox &bounds, BBox &centroidBounds, std::vector<PrimRef> &refs) {
        bounds = BBox();
        centroidBounds = BBox();
        for (int i = start; i < end; i++) {
            BBox tmpBox;
            refs[i].prim->expandBox(tmpBox);
            bounds.add(tmpBox);
            centroidBounds.add(refs[i].centroid);
        }
    }

    BVHNode* buildRecursive(int start, int end, std::vector<PrimRef> &refs, int depth) {
        BVHNode *node = new BVHNode();

        BBox bounds, centroidBounds;
        computeBounds(start, end, bounds, centroidBounds, refs);
        node->box = bounds;

        int nPrims = end - start;
        if (depth >= MAX_DEPTH || nPrims <= MIN_PRIMITIVES) {
            // Leaf
            node->start = start;
            node->end = end;
            return node;
        }

        // Choose axis to split along largest dimension of centroid box
        vec3 diff = centroidBounds.max - centroidBounds.min;
        int axis = 0;
        if (diff.y > diff.x && diff.y > diff.z) axis = 1;
        else if (diff.z > diff.x && diff.z > diff.y) axis = 2;

        float mid = 0.5f * (centroidBounds.min[axis] + centroidBounds.max[axis]);

        // Partition primitives by midpoint
        int midIndex = std::partition(refs.begin() + start, refs.begin() + end, [axis, mid](const PrimRef &p) {
            return p.centroid[axis] < mid;
        }) - refs.begin();

        // If we failed to split (all on one side), just force a split
        if (midIndex == start || midIndex == end) {
            midIndex = (start + end) / 2;
        }

        node->left = buildRecursive(start, midIndex, refs, depth + 1);
        node->right = buildRecursive(midIndex, end, refs, depth + 1);

        return node;
    }

    void build(Purpose purpose) override {
        if (root) {
            clear();
        }

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

        printf("Building%s BVH with %d primitives... ", treePurpose, (int)allPrimitives.size());
        Timer timer;

        // Create references with centroids
        std::vector<PrimRef> refs(allPrimitives.size());
        for (int i = 0; i < (int)allPrimitives.size(); i++) {
            BBox tmpBox;
            allPrimitives[i]->expandBox(tmpBox);
            vec3 c = (tmpBox.min + tmpBox.max) * 0.5f;
            refs[i].prim = allPrimitives[i];
            refs[i].centroid = c;
        }

        root = buildRecursive(0, (int)refs.size(), refs, 0);

        // After building, we no longer need separate refs array, but we must still map the final order of primitives if we reorder them
        // Here we did not reorder allPrimitives themselves, only refs was used for sorting. However, we rely on refs for intersection.
        // To keep it simple, let's store just pointers. The node references [start,end) are in terms of refs array.
        // Let's rearrange allPrimitives according to refs:
        std::vector<Intersectable*> sortedPrims(allPrimitives.size());
        for (int i = 0; i < (int)refs.size(); i++) {
            sortedPrims[i] = refs[i].prim;
        }
        allPrimitives.swap(sortedPrims);

        //printf(" done in %lldms\n", timer.toMs(timer.elapsedNs()));
		printf(" done in %lldms\n", (long long)timer.toMs(timer.elapsedNs()));
    }

    bool intersectNode(BVHNode *node, const Ray& ray, float tMin, float &tMax, Intersection &intersection) {
        if (!node) return false;

        // Test intersection with node's bbox first (already tested by parent except root)
        // We'll do a simple slab test for more robust intersection here
        // But box.testIntersect() suffices (and is already implemented)
        if (!node->box.testIntersect(ray)) {
            return false;
        }

        bool hit = false;
        if (node->isLeaf()) {
            for (int i = node->start; i < node->end; i++) {
                if (allPrimitives[i]->intersect(ray, tMin, tMax, intersection)) {
                    tMax = intersection.t; 
                    hit = true;
                }
            }
        } else {
            // Internal node: check children
            bool hitLeft = intersectNode(node->left, ray, tMin, tMax, intersection);
            bool hitRight = intersectNode(node->right, ray, tMin, tMax, intersection);
            hit = hitLeft || hitRight;
        }

        return hit;
    }

    bool intersect(const Ray& ray, float tMin, float tMax, Intersection &intersection) override {
        if (!root) return false;
        return intersectNode(root, ray, tMin, tMax, intersection);
    }

    bool isBuilt() const override {
        return root != nullptr;
    }

    ~BVHTree() override {
        clear();
    }
};

AcceleratorPtr makeDefaultAccelerator() {
	// TODO: uncomment or add the acceleration structure you have implemented
	//return AcceleratorPtr(new KDTree());
	return AcceleratorPtr(new BVHTree());
	//return AcceleratorPtr(new OctTree());
}

