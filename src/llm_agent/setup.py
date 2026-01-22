from setuptools import find_packages, setup

package_name = 'llm_agent'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'requests'],
    zip_safe=True,
    maintainer='Sankalp',
    maintainer_email='sankalp@example.com',
    description='LLM agent for interpreting parking commands using Qwen 2.5 VL',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'llm_agent_node = llm_agent.llm_agent_node:main',
            'command_interface = llm_agent.command_interface:main',
        ],
    },
)
