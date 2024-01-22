# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- geometry2d - Handles 2D geometry primitives
- se2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input

# Conceptual Questions
1. If you needed to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality

        - Option 1: add a member function in the struct that normalizes the vector, changing its values.
        - Option 2: add a standalone function that takes a vector's value as input and returns a new vector, normalized.
        - Option 3: add a standalone function that takes the reference to the vector, and changes the values in place.

   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.

        - Option 1: Pros: normalizing operation is embedded in the structure. Conceptually, the operation would only be performed on vectors. Cons: the vector is changed in-place, losing the initial vector.
        - Option 2: Pros: we can retain the old vector while getting the normalized version. Cons: more memory is needed as copies of the vector are produced.
        - Option 3: Pros: no extra memory needed for copies of the vector. Cons: the original vector is lost.

   - Which of the methods would you implement and why?

        - Option 2 was implemented, as we might want to keep the original vector, and we dont care about spending a few extra bytes of memory.

2. What is the difference between a class and a struct in C++?

    From a language perspective, the only difference between a class and  struct is the default visibility of their members. A class makes all members private by default, while a struct makes them all public.

3. Why is Vector2D a struct and Transform2D a Class (refer to at least 2 specific C++ core guidelines in your answer)?

    - C.8: Use class rather than struct if any member is non-public
    - C.9: Minimize exposure of members

4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?

    - C.46: By default, declare single-argument constructors explicit, to avoid unintended conversions.


5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer

        - Con.2: By default, make member functions const: a member function should be marked const unless it changes the object’s observable state.

        In this case, the operation `*=` changes the state of the transform, while `inv()` does not.