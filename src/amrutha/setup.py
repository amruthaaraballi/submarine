from setuptools import setup

package_name = 'amrutha'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[
        'submarine',  # ✅ Only this line
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='araballi',
    maintainer_email='your_email@example.com',
    description='Submarine control script',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'submarine = submarine:main',  # ✅ This line matches 'submarine.py'
        ],
    },
)

