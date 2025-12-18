# Feature Specification: Authentication + Personalized User Profiling

**Feature Branch**: `006-user-auth-profiling`
**Created**: 2025-12-17
**Status**: Draft
**Input**: User description: "Add authentication and user profiling so that only logged-in users can access the chatbot, and content can be personalized based on the user's background."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - New User Signup with Profile (Priority: P1)

A new visitor wants to use the AI chatbot assistant. They must create an account and provide their background information (programming level, familiar technologies, hardware access) before they can interact with the chatbot. This ensures every user has a complete profile for personalized assistance.

**Why this priority**: This is the foundational flow - without signup and profiling, no user can access the chatbot. It establishes the core authentication gate and data collection mechanism.

**Independent Test**: Can be fully tested by completing the signup form with profile questions and verifying the user account is created with profile data stored. Delivers value by enabling personalized chatbot access.

**Acceptance Scenarios**:

1. **Given** a visitor on the site without an account, **When** they click the chatbot, **Then** they see a modal prompting them to sign in or sign up
2. **Given** a visitor clicking "Sign Up", **When** they complete email/password fields, **Then** they are presented with the profiling form before account creation completes
3. **Given** a user filling the profiling form, **When** they select their programming level, technologies, and hardware access, **Then** the form validates all required fields are completed
4. **Given** a user submitting a complete signup form with profile, **When** submission succeeds, **Then** the account is created, profile is stored, and user is logged in with chatbot access

---

### User Story 2 - Returning User Sign In (Priority: P1)

A returning user who already has an account wants to access the chatbot. They sign in with their credentials and immediately gain access to the personalized chatbot experience.

**Why this priority**: Equal priority to signup - existing users need seamless access. Critical for user retention and ongoing engagement.

**Independent Test**: Can be tested by signing in with valid credentials and verifying chatbot becomes accessible with user's stored profile.

**Acceptance Scenarios**:

1. **Given** a returning user with an existing account, **When** they click the chatbot while logged out, **Then** they see a modal with Sign In and Sign Up options
2. **Given** a user on the sign-in form, **When** they enter valid email and password, **Then** they are authenticated and gain chatbot access
3. **Given** an authenticated user, **When** they navigate the site, **Then** their session persists across page loads
4. **Given** a user entering invalid credentials, **When** they submit the sign-in form, **Then** they see a clear error message without revealing which field was incorrect

---

### User Story 3 - Authentication Gate for Chatbot (Priority: P1)

The chatbot must be protected behind authentication. Unauthenticated users cannot interact with the AI assistant. This ensures all chatbot interactions are tied to a user profile for personalization.

**Why this priority**: Core security requirement - the chatbot MUST NOT be accessible without authentication. This is a hard gate, not optional.

**Independent Test**: Can be tested by attempting to open the chatbot while logged out and verifying access is blocked with appropriate messaging.

**Acceptance Scenarios**:

1. **Given** an unauthenticated visitor, **When** they click the chatbot icon, **Then** the chatbot does NOT open; instead a modal appears requesting login
2. **Given** an authenticated user with complete profile, **When** they click the chatbot icon, **Then** the chatbot opens normally
3. **Given** an authenticated user with incomplete profile, **When** they click the chatbot icon, **Then** they are redirected to complete their profile first

---

### User Story 4 - Personalized Chatbot Responses (Priority: P2)

When an authenticated user interacts with the chatbot, their profile data (programming level, technologies, hardware access) is sent to the backend. The AI assistant uses this context to tailor responses to the user's experience level and available resources.

**Why this priority**: This is the value-add that makes authentication worthwhile. Without personalization, the auth requirement feels like friction without benefit.

**Independent Test**: Can be tested by asking the same question with different user profiles and verifying responses are tailored to each profile's experience level.

**Acceptance Scenarios**:

1. **Given** a beginner-level user asking about ROS2, **When** the chatbot responds, **Then** the response uses simpler language and provides more foundational context
2. **Given** an advanced user with real robot hardware, **When** they ask about deployment, **Then** the response assumes hardware familiarity and includes practical deployment advice
3. **Given** a user with "simulator only" hardware access, **When** they ask about testing, **Then** the response prioritizes simulation-based approaches

---

### User Story 5 - User Logout (Priority: P3)

An authenticated user wants to log out of their account, ending their session and returning to guest status.

**Why this priority**: Important for security and multi-user devices, but less critical than core auth flows.

**Independent Test**: Can be tested by clicking logout and verifying session is terminated and chatbot access is revoked.

**Acceptance Scenarios**:

1. **Given** an authenticated user, **When** they click the logout button, **Then** their session ends and they return to unauthenticated state
2. **Given** a user who just logged out, **When** they click the chatbot, **Then** they see the authentication modal again

---

### Edge Cases

- What happens when a user's session expires mid-conversation with the chatbot?
  - The chatbot should gracefully handle the expired session and prompt re-authentication
- How does the system handle a user who started signup but abandoned the profiling step?
  - Account creation should be atomic - incomplete profiles mean no account created
- What happens if the backend `/ask` endpoint receives a request without user context?
  - Backend should reject unauthenticated requests with appropriate error response
- How does the system handle users who want to update their profile after initial signup?
  - Profile updates should be supported through a settings/profile page (future enhancement - out of scope for MVP)

## Requirements *(mandatory)*

### Functional Requirements

**Authentication Core**
- **FR-001**: System MUST allow users to create accounts with email and password
- **FR-002**: System MUST validate email format and password strength during signup
- **FR-003**: System MUST authenticate returning users via email and password
- **FR-004**: System MUST persist authentication state across browser sessions
- **FR-005**: System MUST allow users to log out, terminating their session
- **FR-006**: System MUST display clear error messages for authentication failures without revealing sensitive information

**User Profiling**
- **FR-007**: System MUST collect programming level (Beginner/Intermediate/Advanced) during signup
- **FR-008**: System MUST collect familiar technologies via multi-select checkboxes (Python, JS, ROS2, AI/ML, Web, Other) during signup
- **FR-009**: System MUST collect AI/Robotics experience (Yes/No) during signup
- **FR-010**: System MUST collect hardware access level (None/Simulator only/Real robots) during signup
- **FR-011**: System MUST allow optional collection of devices owned (Jetson, Raspberry Pi, Arduino, GPU, Other)
- **FR-012**: System MUST require all mandatory profile fields before account creation completes
- **FR-013**: System MUST persist user profile data associated with user account

**Authentication Gate**
- **FR-014**: Chatbot MUST NOT be accessible to unauthenticated users
- **FR-015**: System MUST display authentication modal when unauthenticated user attempts chatbot access
- **FR-016**: Authentication modal MUST provide both Sign In and Sign Up options
- **FR-017**: System MUST redirect authenticated users with incomplete profiles to profile completion

**User Context Injection**
- **FR-018**: Frontend MUST include authenticated user_id in all `/ask` endpoint requests
- **FR-019**: Frontend MUST include user_profile object in all `/ask` endpoint requests
- **FR-020**: User profile payload MUST include: software_level, technologies array, and hardware_access
- **FR-021**: Backend MUST validate presence of user context before processing chatbot requests

**Atomic Signup**
- **FR-022**: Account creation MUST be atomic; if profile data is incomplete, the user account MUST NOT be created

### Technical Constraints

- Authentication MUST be implemented using Better Auth (https://www.better-auth.com/)
- Custom authentication logic MUST NOT be written
- Better Auth SDK MUST be used for session management and persistence

### RAG Safety Constraint

- Personalization MUST NOT override RAG grounding
- All chatbot responses MUST remain grounded in retrieved content from the knowledge base
- User profile context MUST only influence response tone, complexity, and emphasis - NOT factual content
- The system MUST NOT hallucinate or fabricate information based on user profile assumptions

### API Contract Extension

The `/ask` endpoint request payload MUST conform to this structure:

```
POST /ask
{
  "question": string,
  "context": string | null,
  "top_k": number,
  "user_id": string,
  "user_profile": {
    "programming_level": "beginner" | "intermediate" | "advanced",
    "technologies": string[],
    "hardware_access": "none" | "simulator_only" | "real_robots"
  }
}
```

Field constraints:
- `programming_level`: MUST be one of exactly three values: "beginner", "intermediate", "advanced"
- `technologies`: Array of strings from allowed set: ["Python", "JS", "ROS2", "AI/ML", "Web", "Other"]
- `hardware_access`: MUST be one of exactly three values: "none", "simulator_only", "real_robots"

### Key Entities

- **User**: Represents an authenticated user account; includes email, password hash, creation date, and association to UserProfile
- **UserProfile**: Represents the user's background information; includes programming_level (enum), technologies (array), ai_robotics_experience (boolean), hardware_access (enum), devices_owned (optional array)
- **Session**: Represents an active authentication session; links to User, includes expiration and persistence settings

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can complete full signup (account + profile) in under 3 minutes
- **SC-002**: 95% of returning users successfully sign in on first attempt
- **SC-003**: 100% of chatbot interactions are associated with an authenticated user
- **SC-004**: Zero chatbot responses served to unauthenticated users
- **SC-005**: Authentication state persists correctly across browser sessions for at least 7 days
- **SC-006**: Profile data is successfully passed to backend in 100% of chatbot requests
- **SC-007**: Users with different profiles receive contextually appropriate responses (verified via manual testing with 3 distinct profile types)

## Assumptions

- Better Auth library is compatible with the existing Docusaurus/React frontend
- Existing database infrastructure can accommodate user and profile tables
- Backend FastAPI agent can be extended to accept and utilize user context
- Email/password authentication is sufficient (no OAuth/SSO required for MVP)
- Profile data does not require encryption beyond standard database security
- Users will provide accurate self-assessment of their skill levels
