#pragma once
//
// 兼容头：示例代码沿用原 miao 仓库的 `miao::` 命名空间写法，
// 而本副本的库代码位于 lightning::miao，这里建立别名以保持示例源码不动。
// 本文件通过 -include 选项强制注入到每个编译单元。
//
namespace lightning::miao {}  // 预声明，保证别名不依赖头文件包含顺序
namespace miao = lightning::miao;
