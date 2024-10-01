# Engineering Conventions

## Project Structure

> The project follows a ROS (Robot Operating System) catkin workspace structure. Here's an overview of the project structure:

```
└── 📁src
    └── 📁manager_agent
        └── 📁scripts
            └── manager_agent.py
            └── test_manager_agent.py
        └── 📁test
            └── test_manager_agent.test
        └── CMakeLists.txt
        └── package.xml
    └── 📁multi_agent_system
        └── 📁launch
            └── multi_agent_system.launch
        └── 📁msg
            └── AgentResponse.msg
            └── UserCommand.msg
        └── 📁scripts
            └── agent_responses_logger.py
            └── log_file_path_publisher.py
            └── user_input.py
        └── 📁srv
            └── PlanExecution.srv
            └── StabilityAnalysis.srv
            └── ValidateRequest.srv
        └── CMakeLists.txt
        └── package.xml
    └── 📁planning_agent
        └── 📁launch
        └── 📁scripts
            └── planning_agent.py
        └── 📁test
            └── test_planning_agent.py
        └── CMakeLists.txt
        └── package.xml
    └── 📁stability_agent
        └── 📁scripts
            └── stability_agent.py
        └── CMakeLists.txt
        └── package.xml
    └── 📁structural_engineer_agent
        └── 📁scripts
            └── structural_engineer_agent.py
        └── CMakeLists.txt
        └── package.xml
    └── CMakeLists.txt
```

## Data Structure

> The data is stored in a JSON format. Here's an overview of the data structure:

```
└── 📁data
    └── 📁responses
    └── 📁robot_sequence
    └── disassembly_manuals.json
    └── stability_assessments.json
```


## Conventions
> When writing code, follow these conventions.

- Use comments to explain why you made certain design decisions.
- Regularly refactor code to improve its structure and readability.
- Follow the DRY (Don't Repeat Yourself) principle to minimize code duplication.
- Write modular and reusable code to improve maintainability.
- Use meaningful variable and function names that clearly describe their purpose.
- Implement proper error handling and logging mechanisms.
- Consider edge cases and handle them appropriately in the code.
- Break down complex functions into smaller, more manageable pieces.
- If a function does not have a corresponding test, mention it.
- When building tests, don't mock anything.
- Write simple, verbose code over terse, compact, dense code.
- Make sure you reason step by step through the code and make sure it's correct.
- Double check both your plans and your code for mistakes.