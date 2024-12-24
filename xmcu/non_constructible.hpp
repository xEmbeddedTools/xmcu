#pragma once

/*
 *  Copyright (c) xEmbeddedTools team and contributors.
 *  Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.
 */

namespace xmcu {
class non_constructible
{
protected:
    non_constructible() = delete;
    non_constructible(const non_constructible&) = delete;
    non_constructible(non_constructible&&) = delete;

    non_constructible& operator=(const non_constructible&) = delete;
    non_constructible& operator=(non_constructible&&) = delete;
};
} // namespace xmcu