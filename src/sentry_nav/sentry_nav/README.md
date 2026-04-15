# sentry_nav

## 简介
sentry_nav元包的内部描述子包。

## 功能
- 依赖聚合: 作为sentry_nav元包的组成部分,用于ament依赖声明。
- 构建管理: 作为colcon build的依赖聚合点,确保子包按正确顺序编译。

## 说明
本包不包含实际的源代码或节点逻辑,仅包含 `package.xml` 和 `CMakeLists.txt` 用于项目结构定义。

## 使用
开发者无需直接操作此包,其主要作用是在构建系统层面整合sentry_nav下的各个功能模块。
